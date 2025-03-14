#!/usr/bin/env python3

from breezyslam.algorithms import RMHC_SLAM
#from breezyslam.sensors import Sensor
import numpy as np
import matplotlib.pyplot as plt
import time
import math
from threading import Thread
import ultrasonic_sensor
import dht_sensor
import l298n_act
import fan_act
import pms_sensor
import nion_gen_act
import ctypes
from scipy.ndimage import gaussian_filter

body_lib = ctypes.CDLL("./drivers/body.so")

# macros
STRAIGHT_SPD = 75
TURNING_SPD = 150
MM_PER_SEC = 30

DIST_THRESHOLD = 250
DIST_BOUND = 15

TURN_DEG = 30
TURN_TIME = 0.5

ADJUST_TIME = 0.2



class RobotSLAM:
    """
    Class for managing SLAM with a robot equipped with ultrasonic sensor
    """
    def __init__(self, map_size_pixels=400, map_size_meters=4):
        
        # Initialize sensors, actuators, and SLAM objects
        self.body_lib = body_lib
        if self.body_lib.setup() == 0:
            raise Exception("Failed Initialization!!!")
            
        self.utSensor = ultrasonic_sensor.UltrasonicSensor()
        self.dhtSensor = dht_sensor.DHTSensor()
        self.l298nAct = l298n_act.L298N()
        self.fanSensor = fan_act.Fan()
        self.pmsSensor = pms_sensor.PMSSensor()
        self.nionGen = nion_gen_act.N_Ion_Gen()

        # Create SLAM object
        self.slam = RMHC_SLAM(
            self.utSensor,                # Sensor model
            map_size_pixels,           # Map size in pixels
            map_size_meters,           # Map size in meters
            map_quality=5              # Map quality (lower for faster performance)
        )

        # helps us with orientation
        self.direction = 0
        self.turning = False
        # Initialize map
        self.mapbytes = bytearray(map_size_pixels * map_size_pixels)
        
        # Robot pose (x, y, theta) in mm and degrees
        self.pose = [0, 0, 0]
        
        # Trajectory for visualization
        self.trajectory = [[0, 0]]
        
        # Flag for mapping thread
        self.is_running = False
        
        # Calculate pixel size in mm
        self.pixels_per_meter = map_size_pixels / map_size_meters
        self.map_size_pixels = map_size_pixels
    
        # initialize arrays to store temp, humidity, and pms
        self.temp_data = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)
        self.humidity_data = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)
        self.pms_data = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)

        # reasonable threshold
        self.tempThreshold = 15#30
        self.humThreshold = 60
        self.parThreshold = 350

        # points to revisit
        self.reVisit = []
    
    # used to help update the SLAM algorithm. helps pinpoint where it is
    def update_odometry(self, dt_seconds):
        # Use velocity instead of acceleration
        # came to 350 after testing and measuring on a carpet
        velocity_x = 300  # mm/s - forward velocity
        velocity_y = 0    # mm/s - 0
        if self.turning:
            velocity_x = 0
        # Update the pose angle based on the direction counter
        self.pose[2] = self.direction * TURN_DEG
        
        # Get current orientation
        theta_rad = math.radians(self.pose[2])
        
        # Convert from robot frame to world frame
        world_vel_x = velocity_x * math.cos(theta_rad) - velocity_y * math.sin(theta_rad)
        world_vel_y = velocity_x * math.sin(theta_rad) + velocity_y * math.cos(theta_rad)
        
        # Distance = velocity * time
        dx_mm = world_vel_x * dt_seconds  # mm
        dy_mm = world_vel_y * dt_seconds  # mm
        
        # Update pose
        self.pose[0] += dx_mm
        self.pose[1] += dy_mm
        
        # Save to trajectory (in map coordinates)
        map_x = self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2
        map_y = self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2
        self.trajectory.append([map_x, map_y])
    
    # Turn set number of degrees for a set amount of time
    def turn(self):
        self.turning = True
        self.direction = (self.direction + 1) % (360 / TURN_DEG)
        # turn faster than straight line
        self.l298nAct.turn_right(TURNING_SPD)
        print(f"Turning right. New direction: {self.direction} (degrees: {self.direction * TURN_DEG}°)")
        time.sleep(TURN_TIME)  # Simulate turn time
        self.turning = False

    # left turn
    def left_adjust(self):
        self.l298nAct.drive_right_backward(STRAIGHT_SPD)
        self.l298nAct.drive_left_forward(0)
        time.sleep(ADJUST_TIME)
    
    # stopping the motors
    def pause(self):
        self.l298nAct.stop()
        time.sleep(0.5)

    # read teh sensor data
    def read_sensors(self, map_x, map_y):
        self.l298nAct.stop()
        # wait until we get sensor readings
        while self.dhtSensor.read_dht() < 0:
            time.sleep(1)
        while self.pmsSensor.read_pms() < 0:
            time.sleep(1)
        print(f"Temperature: {self.dhtSensor.temperature} Humidity: {self.dhtSensor.humidity} Concentration of 0.3us particles: {self.pmsSensor.particle}")
        
        self.temp_data[map_x, map_y] = self.dhtSensor.temperature
        self.humidity_data[map_x, map_y] = self.dhtSensor.humidity
        self.pms_data[map_x, map_y] = self.pmsSensor.particle

        if (self.dhtSensor.temperature > self.tempThreshold or
                self.dhtSensor.humidity > self.humThreshold or
                self.pmsSensor.particle > self.parThreshold):
            self.reVisit.append([map_x, map_y])
        #self.l298nAct.drive_forward(self.STRAIGHT_SPD)

    # mapping the room
    def mapping_loop(self):
        last_time = time.time()
        counter=0

        while self.is_running:
            current_time = time.time()
            dt = max(0, current_time - last_time)
            last_time = current_time
            # Get scan from the ultrasonic sensor to mimic a lidar sensor
            distance = self.utSensor.read_ultrasonic()
            print("distance", distance)

            # wait for distance to be populated
            if distance < 0:
                self.pause()
                continue
            counter+=1
            # too close to an obstacle
            if distance < DIST_THRESHOLD:
                # move away then turn
                self.left_adjust()
                self.turn()

            else:
                # add some randomness and a timeout by using counter.
                if counter >= 50:
                    self.l298nAct.drive_right_backward(STRAIGHT_SPD)
                    time.sleep(TURN_TIME)
                    counter = 0
                self.l298nAct.drive_forward(STRAIGHT_SPD)

            scan = self.utSensor.get_scan() 

            self.update_odometry(dt)

            
            # Update SLAM with current scan and odometry
            self.slam.update(scan, pose_change=self.pose)
            
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()

            map_x = int(self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            map_y = int(self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            
            # mapping is inaccurate, so we force it to be within range
            map_x = max(0, min(map_x, self.map_size_pixels - 1))
            map_y = max(0, min(map_y, self.map_size_pixels - 1))

            if 0 <= map_x < self.map_size_pixels and 0 <= map_y < self.map_size_pixels:
                self.read_sensors(map_x, map_y)
                self.l298nAct.drive_forward(STRAIGHT_SPD)
                
            # get the map
            self.slam.getmap(self.mapbytes)
            
            time.sleep(0.1)
    
    #start the SLAM
    def start(self):
        self.is_running = True
        self.mapping_thread = Thread(target=self.mapping_loop)
        self.mapping_thread.daemon = True
        self.mapping_thread.start()
        print("SLAM system started")
        
    # stop the SLAM
    def stop(self):
        self.is_running = False
        self.pmsSensor.close()
        if hasattr(self, 'mapping_thread'):
            self.mapping_thread.join(timeout=1.0)
        if hasattr(self, 'position_thread'):
            self.position_thread.join(timeout=1.0)
        
        print("SLAM system stopped")
    
    # map of where the robot traveled
    def visualize_map(self):
         # Convert the map to a numpy array
         map_array = np.reshape(np.frombuffer(self.mapbytes, dtype=np.uint8),
                               (self.map_size_pixels, self.map_size_pixels))
        
         # Display the map
         plt.figure(figsize=(10, 10))
         plt.imshow(map_array, cmap='gray', origin='lower')
        
         # Plot trajectory
         if len(self.trajectory) > 1:
             trajectory = np.array(self.trajectory)
             plt.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=1)
             plt.plot(trajectory[-1, 0], trajectory[-1, 1], 'ro', markersize=5)
        
         plt.title('Map and Robot Trajectory')
         plt.savefig('map.png')
         plt.show()
    
    def visualizeTemp(self):
        smoothed_temp_data = gaussian_filter(self.temp_data, sigma=5)
        plt.figure(figsize=(10,10))
        heatmap = plt.imshow(smoothed_temp_data, cmap='hot', origin='lower', interpolation='nearest')

        cbar = plt.colorbar(heatmap)
        cbar.set_label('Temperature (°C)')
        plt.title('Temperature Heatmap')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        
        # Save and show the figure
        plt.savefig('temperature_heatmap.png', dpi=300)
        plt.show()
        
    def visualizeHumidity(self):
        # filter out the noise w. a guassian filter
        smoothed_humidity_data = gaussian_filter(self.humidity_data, sigma=5)
        plt.figure(figsize=(10,10))
        humMap = plt.imshow(smoothed_humidity_data, cmap='hot', origin='lower', interpolation='nearest')
        cbar = plt.colorbar(humMap)
        cbar.set_label('Humidity')
        plt.title('Humidity Map')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        
        # Save and show the figure
        plt.savefig('humidity_map.png', dpi=300)
        plt.show()

    def visualizeParticles(self):
        smoothed_pms_data = gaussian_filter(self.pms_data, sigma=5)
        plt.figure(figsize=(10,10))
        partmap = plt.imshow(smoothed_pms_data, cmap='hot', origin='lower', interpolation='nearest')
        cbar = plt.colorbar(partmap)
        cbar.set_label('particles')
        plt.title('Particle Map')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        
        # Save and show the figure
        plt.savefig('particle_map.png', dpi=300)
        plt.show()

# Main function
def main():
    # Create the SLAM system
    slam = RobotSLAM(map_size_pixels=400, map_size_meters=4)
    
    try:
        # Start SLAM with full mapping
        slam.start()
        
        # run mapping for 60 seconds
        print("Mapping in progress...")
        for i in range(40):
            time.sleep(1)
            print(f"Mapping: {i+1}/60 seconds")
        
        # stop mapping loop
        slam.is_running = False
        if hasattr(slam, 'mapping_thread'):
            slam.mapping_thread.join(timeout=5.0)
        if slam.mapping_thread.is_alive():
            # this prevents race conditions with the sensors
            print("Warning: Mapping thread did not terminate properly")
        slam.l298nAct.stop()
        slam.stop()
        time.sleep(1)
        # Visualize the maps
        slam.visualize_map()
        slam.visualizeTemp()
        slam.visualizeHumidity()
        slam.visualizeParticles()

        time.sleep(2.5)
        # check that theres enough places we want to revisit
        numReVisit = len(slam.reVisit)
        if numReVisit > 100:
            print(f"Revisiting for {int(numReVisit/5)}")
            slam.fanSensor.start_fan()
            slam.start()
            for i in range(int(numReVisit/5)):
                time.sleep(1)
                print(f"Cleaning (Mapping): {i+1}/{int(numReVisit/5)} seconds")
            slam.is_running = False
            #slam.nionGen.stop_nion_gen()
            slam.fanSensor.stop_fan()
        
    except KeyboardInterrupt:
        print("Process interrupted by user")
    
    finally:
        # Make sure everything is cleaned up
        slam.is_running = False
        slam.l298nAct.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()

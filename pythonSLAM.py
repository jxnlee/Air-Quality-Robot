#!/usr/bin/env python3

from breezyslam.algorithms import RMHC_SLAM
#from breezyslam.sensors import Sensor
import numpy as np
import matplotlib.pyplot as plt
import time
import math
from threading import Thread
import RPi.GPIO as GPIO  # For interfacing with GPIO on Raspberry Pi
import ultrasonic_sensor
import dht_sensor
import l298n_act
import fan_act
import body_threads

DEFAULT_SPD = 255
spd = 80



class RobotSLAM:
    """
    Class for managing SLAM with a robot equipped with ultrasonic sensor
    """
    def __init__(self, map_size_pixels=800, map_size_meters=20):
        
        # Initialize sensors, actuators, and SLAM objects
        self.robot = body_threads.RobotThreads()
        self.utSensor = ultrasonic_sensor.UltrasonicSensor()
        self.dhtSensor = dht_sensor.DHTSensor()
        self.l298nAct = l298n_act.L298N()
        self.fanSensor = fan_act.Fan()

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
    
        # this is for other readings.
        # Initialize data structures for temperature and humidity
        self.temp_data = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)
        self.humidity_data = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)
        self.pms_data = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)

        # TODO: set it to a sensible value, check more than just temperature
        self.tempThreshold = 5#30
        self.humThreshold = 5
        self.parThreshold = 5
        self.reVisit = []
    
    def update_odometry(self, dt_seconds):
        """
        Update robot pose estimation based on velocity and direction
        This is a simplified odometry model for constant velocity movement
        """
        # Use velocity instead of acceleration
        # came to 350 after testing and measuring on a carpet
        velocity_x = 350  # mm/s - forward velocity
        velocity_y = 0    # mm/s - usually 0 unless you have omnidirectional wheels
        if self.turning:
            velocity_x = 0
        # Update the pose angle based on the direction counter
        self.pose[2] = self.direction * 90 
        
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
    
    #90 degree turn
    def turn(self):
        self.turning = True
        self.direction = (self.direction + 1) % 4
        # turn faster than straight line
        self.l298nAct.turn_right(2*spd)
        print(f"Turning right. New direction: {self.direction} (degrees: {self.direction * 22.5}°)")
        time.sleep(0.5)  # Simulate turn time
        self.turning = False

    def mapping_loop(self):
        last_time = time.time()
        counter=0
        while self.is_running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            # Get scan from the ultrasonic sensor to mimic a lidar sensor
            distance = self.utSensor.read_ultrasonic()
            print("distance", distance)

            if distance == -1:
                continue
            counter+=1
            # too close to an obstacle
            if distance <= 200:
                # move away then turn
                self.l298nAct.drive_left_backward(spd)
                time.sleep(0.1)
                self.turn()
                turnInc = 10
           ## elif distance <= turnInc:
                # turn...
                # increase inc value.
                # try to simulate a spiral
             #   turnInc+=10
              #  self.turn()
            else:
                # add some randomness and a timeout by using counter.
                if counter >= 50:
                    self.l298nAct.drive_right_backward(spd)
                    time.sleep(0.5)
                    counter = 0
                self.l298nAct.drive_forward(spd)

            scan = self.utSensor.get_scan() 

            self.update_odometry(dt)

            
            # Update SLAM with current scan and odometry
            self.slam.update(scan, pose_change=self.pose)
            
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()

            map_x = int(self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            map_y = int(self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            
            index = map_y * self.map_size_pixels + map_x
            # another potential fix, if it just never reads.
            map_x = max(0, min(map_x, self.map_size_pixels - 1))
            map_y = max(0, min(map_y, self.map_size_pixels - 1))
            print(f"mapx: {map_x} mapy: {map_y}")
            if 0 <= map_x < self.map_size_pixels and 0 <= map_y < self.map_size_pixels:
            # Read DHT sensor data
                #dht_data = self.dhtSensor.read_dht()
                print("REACHED TEMPERATURE READING")
                self.l298nAct.stop()
               # self.dhtSensor.read_dht()...might need to always reset to -1 if it is the case that it was moving too fast.
                while self.dhtSensor.temperature == -1:
                    self.dhtSensor.read_dht()
                    time.sleep(1)
                
                # Store the temperature and humidity in the respective maps
                self.temp_data[map_x, map_y] = self.dhtSensor.temperature#dht_data["temperature"]
                self.humidity_data[map_x, map_y] = self.dhtSensor.humidity#dht_data["humidity"]
                print("temp", self.dhtSensor.temperature)
                # || or greater than other values...
                if self.dhtSensor.temperature > self.tempThreshold :
                    self.reVisit.append([map_x, map_y])
                self.l298nAct.drive_forward(spd)
            # Get the map
            self.slam.getmap(self.mapbytes)
            
            time.sleep(0.1)
    
    def start(self):
        """Start the mapping thread"""
        self.is_running = True
        self.mapping_thread = Thread(target=self.mapping_loop)
        self.mapping_thread.daemon = True
        self.mapping_thread.start()
        print("SLAM system started")
        
    def stop(self):
        """Stop the mapping thread"""
        self.is_running = False
        if hasattr(self, 'mapping_thread'):
            self.mapping_thread.join(timeout=1.0)
        if hasattr(self, 'position_thread'):
            self.position_thread.join(timeout=1.0)
        
        print("SLAM system stopped")
    
    def visualize_map(self):
         """Visualize the current map and trajectory"""
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
        
         plt.title('BreezySLAM Map and Robot Trajectory')
         plt.savefig('breezyslam_map.png')
         plt.show()

    def cleanUp(self):
        # might want to start from last index tho.
        i = 0
        print("REVIST:")
        print(self.reVisit)
        while i < len(self.reVisit):
            # just traversing it like this should be close to optimal most of the time, since we're appending to the array, so each location is next to one another.
            x1 = self.reVisit[i][0]  # Fixed indexing here
            y1 = self.reVisit[i][1]  # Fixed indexing here
            self.navigateTo(x1, y1)
            # actuate the fan.
            
            self.fanSensor.start_fan()
            # sleep for a few seconds, can also use while loop to do the thing.
            time.sleep(2)
            self.fanSensor.stop_fan()
            i+=1

    def navigateTo(self, target_x, target_y):  # Added self parameter and renamed to avoid confusion
        # Get current position in map coordinates
        map_x = int(self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
        map_y = int(self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
        
        print(f"Navigating from ({map_x}, {map_y}) to ({target_x}, {target_y})")
        
        # First handle x-coordinate navigation
        startTime  = time.time()
        while abs(map_x - target_x) > 100:  # Using a threshold of 5 pixels'
            currTime = time.time()
            if startTime - currTime > 12:
                break
            # read ultrasonic, if something is in front, turn right twice, then move forward for half a second, until
            distance = self.utSensor.read_ultrasonic()
            if distance == -1:
                continue
            if distance < 200:
                self.l298nAct.drive_left_backward(spd)
                # time.sleep(0.2)
                # self.l298nAct.stop()
            if map_x < target_x:
                # Need to move right (east)
                target_direction = 0  # East
            else:
                # Need to move left (west)
                target_direction = 2  # West
                
            # Turn to the target direction
            while self.direction != target_direction:
                self.turn()
                
            # Drive forward
            self.l298nAct.drive_forward(spd)
            # time.sleep(0.5)  # Drive for a short time
            # self.l298nAct.stop()
            
            # Update current position
            map_x = int(self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            map_y = int(self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            time.sleep(0.1)
        
        startTime  = time.time()
        # Then handle y-coordinate navigation
        while abs(map_y - target_y) > 100:  
            currTime = time.time()
            if startTime - currTime > 12:
                break
            distance = self.utSensor.read_ultrasonic()
            if distance == -1:
                continue
            if distance < 200:
                self.l298nAct.drive_left_backward(spd)
                time.sleep(0.2)
                # self.l298nAct.stop()
            if map_y < target_y:
                # Need to move up (north)
                target_direction = 1  # North
            else:
                # Need to move down (south)
                target_direction = 3  # South
                
            # Turn to the target direction
            while self.direction != target_direction:
                self.turn()
                
            # Drive forward
            self.l298nAct.drive_forward(spd)
            time.sleep(0.5)  # Drive for a short time
            # self.l298nAct.stop()
            
            # Update current position
            map_x = int(self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            map_y = int(self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            time.sleep(0.1)

        print(f"Reached destination ({target_x}, {target_y})")


    def position_tracking_loop(self):
        """Track position without mapping new areas"""   
        last_time = time.time()
     
        while self.is_running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            # Get scan from the ultrasonic sensor
            scan = self.utSensor.get_scan() 
            # time
            self.update_odometry(dt)
            # Update SLAM with current scan and odometry
            self.slam.update(scan, pose_change=self.pose)
            
            # Get current pose estimate from SLAM
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()
            
            # Update trajectory for visualization
            map_x = self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2
            map_y = self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2
            self.trajectory.append([map_x, map_y])
            
            time.sleep(0.1)  # Run at 10Hz


# Main function
def main():
    # Create the SLAM system
    slam = RobotSLAM(map_size_pixels=800, map_size_meters=20)
    
    try:
        # Start SLAM with full mapping
        slam.start()
        
        # Run mapping for 60 seconds
        print("Mapping in progress...")
        for i in range(40):
            time.sleep(1)
            print(f"Mapping: {i+1}/60 seconds")
        
        # Stop mapping loop
        slam.is_running = False
        if hasattr(slam, 'mapping_thread'):
            slam.mapping_thread.join(timeout=1.0)
        
        # Visualize the map
        slam.visualize_map()
        
        # Start just position tracking (not full mapping)
        slam.is_running = True
        slam.position_thread = Thread(target=slam.position_tracking_loop)
        slam.position_thread.daemon = True
        slam.position_thread.start()
        
        # Now navigate to important points
        print("Navigating to hotspots...")
        slam.cleanUp()
        
        # Stop position tracking
        slam.is_running = False
        slam.position_thread.join(timeout=1.0)
        
    except KeyboardInterrupt:
        print("Process interrupted by user")
    
    finally:
        # Make sure everything is cleaned up
        slam.is_running = False
        slam.l298nAct.stop()  # Stop motors
        GPIO.cleanup()


if __name__ == "__main__":
    main()

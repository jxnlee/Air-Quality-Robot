#!/usr/bin/env python3

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Sensor
import numpy as np
import matplotlib.pyplot as plt
import time
import math
from threading import Thread
import RPi.GPIO as GPIO  # For interfacing with GPIO on Raspberry Pi


class RobotSLAM:
    """
    Class for managing SLAM with a robot equipped with ultrasonic sensor
    """
    def __init__(self, map_size_pixels=800, map_size_meters=20):
        
        # Initialize sensor and SLAM objects
        self.sensor = UltrasonicSensor()
        
        # Create SLAM object
        self.slam = RMHC_SLAM(
            self.sensor,                # Sensor model
            map_size_pixels,           # Map size in pixels
            map_size_meters,           # Map size in meters
            map_quality=5              # Map quality (lower for faster performance)
        )
        
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
        dht_data = bytearray(map_size_pixels * map_size_pixels)

    
    def update_odometry(self, dt_seconds):
        """
        Update robot pose estimation based on accelerometer data
        This is a very simplified odometry model
        """
        # not sure how to port it
        accel_x, accel_y, _ = read_accelerometer()
        
        # Get current orientation
        theta_rad = math.radians(self.pose[2])
        
        # Convert from robot frame to world frame
        world_accel_x = accel_x * math.cos(theta_rad) - accel_y * math.sin(theta_rad)
        world_accel_y = accel_x * math.sin(theta_rad) + accel_y * math.cos(theta_rad)
        
        # Basic physics: distance = 0.5 * a * t^2
        dx_mm = 0.5 * world_accel_x * dt_seconds * dt_seconds * 1000  # Convert to mm
        dy_mm = 0.5 * world_accel_y * dt_seconds * dt_seconds * 1000  # Convert to mm
        
        # Update pose
        self.pose[0] += dx_mm
        self.pose[1] += dy_mm
        
        # For a more accurate implementation, you would:
        # 1. Use a proper motion model
        # 2. Include wheel encoder data if available
        # 3. Implement a complementary or Kalman filter
        
        # Save to trajectory (in map coordinates)
        map_x = self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2
        map_y = self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2
        self.trajectory.append([map_x, map_y])
    
    def mapping_loop(self):
        """Main loop for mapping"""
        last_time = time.time()
        
        while self.is_running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Update odometry based on accelerometer
            self.update_odometry(dt)
            
            # Get scan from the ultrasonic sensor
            scan = self.sensor.get_scan()
            
            # Update SLAM with current scan and odometry
            self.slam.update(scan, pose_change=self.pose)
            
            # Get current pose estimate from SLAM
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()

            # Get current position in map coordinates
            map_x = int(self.pose[0] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            map_y = int(self.pose[1] / 1000 * self.pixels_per_meter + self.map_size_pixels // 2)
            
            # Calculate index in the flat array
            index = map_y * self.map_size_pixels + map_x
            
            # TODO: either create multiple arrays or use a 3d array or a numpy array to hold multiple data.         
            dht_value = self.read_dht()
            
            # Store the value in your parallel array
            if 0 <= index < len(self.sensor_data):
                self.dht_data[index] = dht_value
            # Get the map
            self.slam.getmap(self.mapbytes)
            
            time.sleep(0.1)  # Run at 10Hz
    
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
        
        # Clean up GPIO
        GPIO.cleanup()
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


# Main function
def main():
    # Create the SLAM system
    slam = RobotSLAM(map_size_pixels=800, map_size_meters=20)
    
    try:
        # Start SLAM
        slam.start()
        
        # Run for 60 seconds
        print("Running BreezySLAM for 60 seconds...")
        for i in range(60):
            time.sleep(1)
            print(f"Mapping in progress... {i+1}/60 seconds")
        
    except KeyboardInterrupt:
        print("Mapping interrupted by user")
    
    finally:
        # Stop SLAM
        slam.stop()
        
        # Visualize the map
        slam.visualize_map()


if __name__ == "__main__":
    main()
import ctypes

from breezyslam.sensors import Sensor

class UltrasonicSensor(Sensor):
    def __init__(self):
        # Initialize as a Sensor for BreezySLAM with parameters:
        # detection_angle_degrees: total field of view in degrees
        # distance_no_detection_mm: maximum distance reading
        # detection_margin: offset from distance measurement
        # offset_mm: sensor position offset
        Sensor.__init__(self, 
                       detection_angle_degrees=30,  # Ultrasonic FOV
                       distance_no_detection_mm=5000,  # Max distance to report
                       detection_margin=70,  # Offset from actual distance
                       offset_mm=0)  # Sensor position offset
        
        # Load the C library
        self.ultrasonic_lib = ctypes.CDLL("./libultrasonic.so")
        self.ultrasonic_lib.read_ultrasonic.argtypes = [ctypes.POINTER(ctypes.c_ulong)]
        self.ultrasonic_lib.read_ultrasonic.restype = None
        self.distance = 0

    def read_ultrasonic(self, distance_ref=None):
        # Create a ctypes float to hold the result
        distance_c = ctypes.c_ulong()
        
        # Call the C function
        result = self.ultrasonic_lib.read_ultrasonic(ctypes.byref(distance_c))
        
        # Get the value from the ctypes object
        distance_value = distance_c.value
        
        # If a reference was provided, update it
        if distance_ref is not None:
            distance_ref[0] = distance_value
        
        # Return the distance value
        return distance_value

def get_scan(self):

    # Get the distance measurement
    distance = self.read_ultrasonic()
    
    # Number of rays in the scan (should match SLAM configuration)
    num_rays = 360  # Adjust based on your SLAM setup
    
    # Create an array of distances
    max_distance = 5000  # Maximum distance in mm
    scan = [max_distance] * num_rays
    
    # Define the sensor's field of view
    center = num_rays // 2
    field_of_view = 30  # Degrees
    
    # Fill in the actual reading in the sensor's field of view
    start_idx = center - (field_of_view // 2)
    end_idx = center + (field_of_view // 2)
    
    for i in range(start_idx, end_idx + 1):
        scan[i % num_rays] = distance
    
    return scan

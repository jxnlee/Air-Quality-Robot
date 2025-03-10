import ctypes

from breezyslam.sensors import Laser

class UltrasonicSensor(Laser):
    """
    Ultrasonic sensor implementation for BreezySLAM
    """
    def __init__(self):
        # Initialize with parameters suitable for an ultrasonic sensor
        # scan_size, scan_rate_hz, detection_angle_degrees, distance_no_detection_mm, detection_margin, offset_mm
        Laser.__init__(self, 360, 10, 30, 5000, 0, 0)
        
        # Load the ultrasonic library
        self.ultrasonic_lib = ctypes.CDLL("./ultrasonic.so")
        self.ultrasonic_lib.read_ultrasonic.argtypes = [ctypes.POINTER(ctypes.c_ulong)]
        self.ultrasonic_lib.read_ultrasonic.restype = None
        #self.ultrasonic_lib.init_ultrasonic()
        self.distance = 0
        
    def read_ultrasonic(self):
        # Create a ctypes ulong to hold the result
        distance_c = ctypes.c_ulong()
        
        # Call the C function
        self.ultrasonic_lib.read_ultrasonic(ctypes.byref(distance_c))
        
        # Get the value from the ctypes object
        self.distance = distance_c.value
        
        return self.distance
        
    def get_scan(self):
        """
        Returns a 360-degree scan simulated from a single ultrasonic reading
        """
        # Create an array of distances
        scan = [self.distance_no_detection_mm] * self.scan_size
        
        # Define the sensor's field of view
        center = self.scan_size // 2
        field_of_view = int(self.detection_angle_degrees)
        
        # Fill in the actual reading in the sensor's field of view
        half_fov = field_of_view // 2
        start_idx = (center - half_fov) % self.scan_size
        end_idx = (center + half_fov) % self.scan_size
        
        for i in range(start_idx, end_idx + 1):
            scan[i % self.scan_size] = self.distance
        
        return scan

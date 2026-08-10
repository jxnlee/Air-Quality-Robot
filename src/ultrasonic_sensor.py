import ctypes

# inherit the Laser class, used for Lidar
from breezyslam.sensors import Laser

class UltrasonicSensor(Laser):
    """
    Ultrasonic sensor implementation for BreezySLAM
    """
    def __init__(self):

        Laser.__init__(self, 360, 10, 30, 5000, 0, 0)
        
        self.ultrasonic_lib = ctypes.CDLL("./drivers/ultrasonic.so")
        self.ultrasonic_lib.read_ultrasonic.argtypes = [ctypes.POINTER(ctypes.c_long)]
        self.ultrasonic_lib.read_ultrasonic.restype = None
        self.ultrasonic_lib.init_ultrasonic()
        self.distance = 0
        
    def read_ultrasonic(self):

        distance_c = ctypes.c_long()
        
        # calls the c function
        self.ultrasonic_lib.read_ultrasonic(ctypes.byref(distance_c))
        
        # get value from the ctypes object
        self.distance = distance_c.value
        
        return self.distance
        
    # mimic 360 view by giving it the distance data. needed to pass into SLAM object
    def get_scan(self):
        # array of distances
        scan = [self.distance_no_detection_mm] * self.scan_size
        
        # define the sensor's field of view
        center = self.scan_size // 2
        field_of_view = int(self.detection_angle_degrees)
        
        # fill in the reading in the sensor's field of view
        half_fov = field_of_view // 2
        start_idx = (center - half_fov) % self.scan_size
        end_idx = (center + half_fov) % self.scan_size
        
        for i in range(start_idx, end_idx + 1):
            scan[i % self.scan_size] = self.distance
        
        return scan

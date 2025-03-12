import ctypes

pms_lib = ctypes.CDLL("./pms.so")

# Define the PM25_Data structure to match the C structure
class PM25_Data(ctypes.Structure):
    _fields_ = [
        ("pm10_standard", ctypes.c_uint16),
        ("pm25_standard", ctypes.c_uint16),
        ("pm100_standard", ctypes.c_uint16),
        ("pm10_env", ctypes.c_uint16),
        ("pm25_env", ctypes.c_uint16),
        ("pm100_env", ctypes.c_uint16),
        ("particles_03um", ctypes.c_uint16),
        ("particles_05um", ctypes.c_uint16),
        ("particles_10um", ctypes.c_uint16),
        ("particles_25um", ctypes.c_uint16),
        ("particles_50um", ctypes.c_uint16),
        ("particles_100um", ctypes.c_uint16),
        ("unused", ctypes.c_uint16),
        ("checksum", ctypes.c_uint16),
        ("aqi_pm25_us", ctypes.c_uint16),
        ("aqi_pm100_us", ctypes.c_uint16)
    ]

# Set the function signatures
pms_lib.init_pms.argtypes = []
pms_lib.init_pms.restype = ctypes.c_int

pms_lib.read_pms.argtypes = [ctypes.POINTER(ctypes.c_uint16)]
pms_lib.read_pms.restype = ctypes.c_int

pms_lib.print_pms_readings.argtypes = []
pms_lib.print_pms_readings.restype = None

pms_lib.close_pms.argtypes = []
pms_lib.close_pms.restype = None

class PMSSensor:
    def __init__(self):
        self.pms_lib = pms_lib
        self.particle = -1
        # Initialize the sensor
        result = self.pms_lib.init_pms()
        if result != 1:
            raise Exception("Failed to initialize PMS sensor")

    def read_pms(self):
        part = ctypes.c_uint16()
        result = self.pms_lib.read_pms(ctypes.byref(part))
        if result == 1:
            self.particle = part.value
            return 1
        return 0  # Return 0 for failures, not -1 to match C function

    def print_pms_readings(self):
        self.pms_lib.print_pms_readings()
        
    def close(self):
        self.pms_lib.close_pms()
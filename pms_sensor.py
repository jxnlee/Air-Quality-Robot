import ctypes


pms_lib = ctypes.CDLL("./pms.so")

pms_lib.read_pms.argtypes = [ctypes.POINTER(ctypes.c_uint16)]
pms_lib.read_pms.restype = ctypes.c_int

pms_lib.print_pms_readings.argtypes = []
pms_lib.print_pms_readings.restype = None

class PMSSensor:
    def __init__(self):
        self.pms_lib = pms_lib
        self.particle = -1

    def read_pms(self):
        part = ctypes.c_uint16()
        result = self.pms_lib.read_pms(ctypes.byref(part))
        if result == 1:
            self.particle = part.value
            return 1
        # else:
        #     raise Exception("Failed to read PMS sensor data")
        return -1

    def print_pms_readings(self):
        self.pms_lib.print_pms_readings()
import ctypes


pms_lib = ctypes.CDLL("./pms.so")

pms_lib.read_pms.argtypes = [ctypes.POINTER(ctypes.c_uint16)]
pms_lib.read_pms.restype = ctypes.c_int

pms_lib.read_pms.argtypes = []
pms_lib.read_pms.restype = None

class PMSSensor:
    def __init__(self):
        self.pms_lib = pms_lib
        self.particle = -1

    def read_pms(self):
        particle = ctypes.c_uint16()
        result = self.pms_lib.read_pms(ctypes.byref(particle))
        if result == 1:
            self.particle = particle.value
        else:
            raise Exception("Failed to read PMS sensor data")

    def print_pms_readings(self):
        self.pms_lib.print_pms_readings()
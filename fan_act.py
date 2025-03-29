# wrapper for C Fan Actuator
import ctypes

fan_lib = ctypes.CDLL("./drivers/fan.so")

# function signatures
fan_lib.start_fan.argtypes = []
fan_lib.start_fan.restype = None

fan_lib.stop_fan.argtypes = []
fan_lib.stop_fan.restype = None

class Fan:
    def __init__(self):
        self.fan_lib = fan_lib

    def start_fan(self):
        self.fan_lib.start_fan()
    
    def stop_fan(self):
        self.fan_lib.stop_fan()
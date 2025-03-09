import ctypes
import time

l298n_lib = ctypes.CDLL("./l298n.so")

# Define argument types and return types for the C functions
l298n_lib.set_motors_speed.argtypes = [ctypes.c_uint8]
l298n_lib.set_motors_speed.restype = None

l298n_lib.motors_off.argtypes = []  # This is correct for functions with no parameters
l298n_lib.motors_off.restype = None

l298n_lib.drive_left.argtypes = [ctypes.c_int]
l298n_lib.drive_left.restype = None

l298n_lib.drive_right.argtypes = [ctypes.c_int]
l298n_lib.drive_right.restype = None

l298n_lib.drive_forward.argtypes = [ctypes.c_int]
l298n_lib.drive_forward.restype = None

l298n_lib.drive_backward.argtypes = [ctypes.c_int]
l298n_lib.drive_backward.restype = None


class L298N:
    def __init__(self):
        self.l298n_lib = l298n_lib
        self.l298n_lib.init_l298n()

    def set_motors_speed(self, speed):
        if not 0 <= speed <= 255:
            raise ValueError("Speed must be between 0 and 255")
        speed_uint8 = ctypes.c_uint8(speed)
        self.l298n_lib.set_motors_speed(speed_uint8)
    
    def motors_off(self):
        self.l298n_lib.motors_off()
    
    def drive_left(self, duration):
        self.l298n_lib.drive_left(ctypes.c_int(duration))
    
    def drive_right(self, duration):
        self.l298n_lib.drive_right(ctypes.c_int(duration))
    
    def drive_forward(self, duration):
        self.l298n_lib.drive_forward(ctypes.c_int(duration))
    
    def drive_backward(self, duration):
        self.l298n_lib.drive_backward(ctypes.c_int(duration))
    
    # for turning
    def turn_left(self, duration, speed=128):
        self.set_motors_speed(speed)
        self.drive_left(duration)
    
    def turn_right(self, duration, speed=128):
        self.set_motors_speed(speed)
        self.drive_right(duration)
    
    def move_forward(self, duration, speed=128):
        self.set_motors_speed(speed)
        self.drive_forward(duration)
    
    def move_backward(self, duration, speed=128):
        self.set_motors_speed(speed)
        self.drive_backward(duration)
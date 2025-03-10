import ctypes
import time

l298n_lib = ctypes.CDLL("./l298n.so")

# Define argument types and return types for the C functions
l298n_lib.set_motors_speed.argtypes = [ctypes.c_uint8]
l298n_lib.set_motors_speed.restype = None

l298n_lib.motors_off.argtypes = []  # This is correct for functions with no parameters
l298n_lib.motors_off.restype = None

l298n_lib.drive_left_forward.argtypes = [ctypes.c_uint8]
l298n_lib.drive_left_forward.restype = None

l298n_lib.drive_left_backward.argtypes = [ctypes.c_uint8]
l298n_lib.drive_left_backward.restype = None

l298n_lib.drive_right_forward.argtypes = [ctypes.c_uint8]
l298n_lib.drive_right_forward.restype = None

l298n_lib.drive_right_backward.argtypes = [ctypes.c_uint8]
l298n_lib.drive_right_backward.restype = None

l298n_lib.drive_forward.argtypes = [ctypes.c_uint8]
l298n_lib.drive_forward.restype = None

l298n_lib.drive_backward.argtypes = [ctypes.c_uint8]
l298n_lib.drive_backward.restype = None


class L298N:
    def __init__(self):
        self.l298n_lib = l298n_lib
        #self.l298n_lib.init_l298n()

    def set_motors_speed(self, speed):
        if not 0 <= speed <= 255:
            raise ValueError("Speed must be between 0 and 255")
        speed_uint8 = ctypes.c_uint8(speed)
        self.l298n_lib.set_motors_speed(speed_uint8)
    
    def stop(self):
        self.l298n_lib.motors_off()
    
    def drive_left_forward(self, speed):
        self.l298n_lib.drive_left_forward(ctypes.c_uint8(speed))
    def drive_left_backward(self, speed):
        self.l298n_lib.drive_left_backward(ctypes.c_uint8(speed))
    def drive_right_forward(self, speed):
        self.l298n_lib.drive_right_forward(ctypes.c_uint8(speed))
    def drive_right_backward(self, speed):
        self.l298n_lib.drive_right_backward(ctypes.c_uint8(speed))
    
    def drive_forward(self, speed):
        self.l298n_lib.drive_forward(ctypes.c_uint8(speed))
    
    def drive_backward(self, speed):
        self.l298n_lib.drive_backward(ctypes.c_uint8(speed))
    
    # for turning
    def turn_left(self, speed=255):
        self.drive_left_forward(speed)
        self.drive_right_forward(int(speed/4))
    
    def turn_right(self, speed=255):
        self.drive_right_forward(speed)
        self.drive_left_forward(int(speed/4))
    
    def move_forward(self, speed=255):
        self.drive_forward(speed)
    
    def move_backward(self, speed=255):
        self.drive_backward(speed)

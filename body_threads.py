import ctypes

body_lib = ctypes.CDLL("./body.so")

class RobotThreads:
    def __init__(self):
        self.body_lib = body_lib
        self.body_lib.setup()

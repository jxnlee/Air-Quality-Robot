# wrapper for C nion generator
import ctypes

nion_gen_lib = ctypes.CDLL("./drivers/nion_gen.so")

nion_gen_lib.start_nion_gen.argtypes = []
nion_gen_lib.start_nion_gen.restype = None

nion_gen_lib.stop_nion_gen.argtypes = []
nion_gen_lib.stop_nion_gen.restype = None



class N_Ion_Gen:
    def __init__(self):
        self.nion_gen_lib = nion_gen_lib

    def start_nion_gen(self):
        self.nion_gen_lib.start_nion_gen()
    
    def stop_nion_gen(self):
        self.nion_gen_lib.stop_nion_gen()
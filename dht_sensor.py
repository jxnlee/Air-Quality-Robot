import ctypes


dht_lib = ctypes.CDLL("./drivers/dht.so")

dht_lib.read_dht.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
dht_lib.read_dht.restype = ctypes.c_int

class DHTSensor:
    def __init__(self):
        self.dht_lib = dht_lib
        self.temperature = -1
        self.humidity = -1

    def read_dht(self):
        temp = ctypes.c_float()
        hum = ctypes.c_float()
        # pass in temp and hum
        result = dht_lib.read_dht(ctypes.byref(temp), ctypes.byref(hum))

        # success
        if result == 1:
            self.temperature = temp.value
            self.humidity = hum.value
            return 1
        else:
            return -1

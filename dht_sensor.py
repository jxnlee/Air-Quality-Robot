import ctypes


dht_lib = ctypes.CDLL("./libdht.so")

dht_lib.read_dht.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
dht_lib.read_dht.restype = ctypes.c_int

class DHTSensor:
    def __init__(self):
        pass

    def read_dht():
        """Calls the C function to get temperature and humidity."""
        temperature = ctypes.c_float()
        humidity = ctypes.c_float()
        
        result = dht_lib.read_dht(ctypes.byref(temperature), ctypes.byref(humidity))

        if result == 1:
            return {"temperature": temperature.value, "humidity": humidity.value}
        else:
            raise Exception("Failed to read DHT sensor data")
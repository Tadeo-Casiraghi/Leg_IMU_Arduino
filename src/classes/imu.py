import numpy as np

class IMU:

    def __init__(self):
        self.settings = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
        self.data = {}
        self.times = np.array([])
        for setting in self.settings:
            self.data[setting] = np.array([])
        self.gyro_scale = (250/16384) * np.pi/360
        self.accel_scale = 9.81/2048
        
    def decode_data(self, data):
        pointer = 0
        for setting in self.settings:
            if 'a' in setting:
                scale = self.accel_scale
            else:
                scale = self.gyro_scale
            if data[pointer] >= 128:
                value = ((data[pointer]<<8)+ data[pointer+1]) - 2**16
            else:
                value = (data[pointer]<<8)+ data[pointer+1]
            self.data[setting] = np.append(self.data[setting], value * scale )
            pointer += 2
    
    def decode_time(self, data):
        value = (data[0]<<24) + (data[0+1]<<16) + (data[0+2]<<8) + (data[0+3])
        self.times = np.append(self.times, value/1000000)
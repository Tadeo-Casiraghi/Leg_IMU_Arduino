import numpy as np

class IMU:

    def __init__(self):
        self.settings = ['a', 'g']
        self.accel = []
        self.gyro = []
        self.calib_accel = []
        self.calib_gyro = []
        self.times = np.array([])
        self.calib_times = np.array([])
        self.gyro_scale = (250/16384) * np.pi/360
        self.accel_scale = 9.81/2048
        
    def decode_data(self, data, calibration = False):
        pointer = 0
        for setting in self.settings:
            temp = np.array([0,0,0]).T
            for i in range(3):
                if data[pointer] >= 128:
                    value = ((data[pointer]<<8)+ data[pointer+1]) - 2**16
                else:
                    value = (data[pointer]<<8)+ data[pointer+1]
                temp[i] = value
                pointer += 2

            if calibration and 'a' in setting:
                self.calib_accel.append(temp * self.accel_scale )
            elif calibration:
                self.calib_gyro.append(temp * self.gyro_scale)
            elif 'a' in setting:
                self.accel.append(temp * self.accel_scale)
            else:
                self.gyro.append(temp * self.gyro_scale)
            
    
    def decode_time(self, data, calibration = False):
        value = (data[0]<<24) + (data[0+1]<<16) + (data[0+2]<<8) + (data[0+3])
        if calibration:
            self.calib_times = np.append(self.calib_times, value/1000000)
        else:
            self.times = np.append(self.times, value/1000000)
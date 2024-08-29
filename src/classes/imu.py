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
        self.gyro_scale = (500/16384) * np.pi/360
        self.accel_scale = 9.81/4096
        
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

    def clean_all(self,):
        self.gyro = self.clean_signal(self.gyro, 4*16348*self.gyro_scale)
        self.calib_gyro = self.clean_signal(self.calib_gyro, 4*16348*self.gyro_scale)
        self.accel = self.clean_signal(self.accel, 4*16348*self.accel_scale)
        self.calib_accel = self.clean_signal(self.calib_accel, 4*16348*self.accel_scale)

    def clean_signal(self, signal, total):
        clean = [signal[0]]
        adder = [0]*3
        for sig0, sig1 in zip(signal[0:-1], signal[1:]):
            temp = [0]*3
            for axis in range(3):
                if sig1[axis] - sig0[axis] > total*0.5:
                    adder[axis] -= total
                elif sig1[axis] - sig0[axis] < -total*0.5:
                    adder[axis] += total
                temp[axis] = sig1[axis] + adder[axis]
            clean.append(np.array(temp))
        return clean

    def decode_time(self, data, calibration = False):
        value = (data[0]<<24) + (data[0+1]<<16) + (data[0+2]<<8) + (data[0+3])
        if calibration:
            self.calib_times = np.append(self.calib_times, value/1000000)
        else:
            self.times = np.append(self.times, value/1000000)
from classes.madgwickahrs import MadgwickAHRS
from classes.quaternion import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import glob
import os
from classes.imu import IMU

class IMUinterpreter:
    def __init__(self, filename = None, file_number = -1):
        if filename == None:
            current_directory = os.path.dirname(os.path.abspath(__file__))
            # Go up one directory
            parent_directory = os.path.dirname(current_directory)
            # Go into the "data" directory
            data_directory = os.path.join(parent_directory, 'data')
            list_of_files = glob.glob(os.path.join(data_directory,'*.bin'))
            list_of_files.sort()
            # TODO: use calibration and other data
            calib_filename = list_of_files[file_number -1]
            filename = list_of_files[file_number]
        
        with open(calib_filename,'rb') as file:
            self.calib_data = file.read()

        with open(filename,'rb') as file:
            self.raw = file.read()

        self.packetsize = 12*3+4
        self.imus = []

        for _ in range(3):
            self.imus.append(IMU())

        self.decode(self.raw)
        self.decode(self.calib_data, calibration = True)

        self.times = self.imus[0].times
        self.calib_times = self.imus[0].calib_times
        

    def decode(self, data, calibration = False):
        pointer = 0
        while pointer < len(data)-self.packetsize:
            for imu in self.imus:
                imu.decode_data(data[pointer:(pointer + self.packetsize - 4)], calibration)
            pointer += self.packetsize - 4
            for imu in self.imus:
                imu.decode_time(self.raw[pointer:(pointer+4)], calibration)
            pointer += 4
        
    def get_gdot(self, g, t):
        dt = np.average(np.diff(t))
        gdot = []
        for g0, g1, g2, g3 in zip(g[:-4], g[1:-3], g[3:-1], g[4:]):
            gdot.append((g0 - 8*g1 + 8*g2 - g3)/(12*dt))
        return np.array(gdot)

    def generate_dataset(self, a1, a2, g1, g2, g1dot, g2dot, t):
        S = [[a1[2], a2[2], g1[2], g2[2], g1dot[0], g2dot[0]]]
        tprev = t[2]
        for index, tim in enumerate(t[2:-2]):
            if tim - tprev > 0.1:
                S.append([a1[2+index], a2[2+index], g1[2+index], g2[2+index], g1dot[index], g2dot[index]])
                tprev = tim
        return S
    
    def calibrate_pair(self, a1, a2, g1, g2, g1dot, g2dot, t):
        S = self.generate_dataset(a1, a2, g1, g2, g1dot, g2dot, t)
        # TODO: continue here
    
    def calibrate(self,):
        self.gdots = []
        for imu in self.imus:
            self.gdots.append(self.get_gdot(imu.calib_gyro, imu.calib_times))
        
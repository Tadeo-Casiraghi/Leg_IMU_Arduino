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

            filename = list_of_files[file_number]
        
        with open(filename,'rb') as file:
            self.raw = file.read()

        self.packetsize = 12*3+4
        self.imus = []

        for _ in range(3):
            self.imus.append(IMU())

        pointer = 0

        while pointer < len(self.raw)-self.packetsize:
            for imu in self.imus:
                imu.decode_data(self.raw[pointer:(pointer + self.packetsize - 4)])
            pointer += self.packetsize - 4
            for imu in self.imus:
                imu.decode_time(self.raw[pointer:(pointer+4)])
            pointer += 4
        
        self.times = self.imus[0].times
from lib2to3.pytree import convert
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
                imu.decode_data(data[pointer:(pointer + 12)], calibration)
                pointer += 12
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
        S = [[a1[2]], [a2[2]], [g1[2]], [g2[2]], [g1dot[0]], [g2dot[0]]]
        tprev = t[2]
        for index, tim in enumerate(t[2:-2]):
            if tim - tprev > 0.1:
                S[0].append(a1[2+index])
                S[1].append(a2[2+index])
                S[2].append(g1[2+index])
                S[3].append(g2[2+index])
                S[4].append(g1dot[index])
                S[5].append(g2dot[index])
                tprev = tim
        return S

    def calc_j(self, x):
        j1 = np.array([np.cos(x[0])*np.cos(x[1]),
                       np.cos(x[0])*np.sin(x[1]),
                       np.sin(x[0])]).T
        j2 = np.array([np.cos(x[2])*np.cos(x[3]),
                       np.cos(x[2])*np.sin(x[3]),
                       np.sin(x[2])]).T
        return j1, j2
    
    def calc_error1(self, g1, g2, j1, j2):
        error = []
        for g1k, g2k in zip(g1, g2):
            error.append(np.linalg.norm(np.cross(g1k, j1)) - np.linalg.norm(np.cross(g2k, j2)))
        return error
    
    def calc_deriv_phi(self, phi, tita):
        return np.array([-np.sin(phi) * np.cos(tita),
                         -np.sin(phi) * np.sin(tita),
                          np.cos(phi)]).T
    
    def calc_deriv_tita(self, phi, tita):
        return np.array([-np.cos(phi) * np.sin(tita),
                          np.cos(phi) * np.cos(tita),
                          0]).T
    
    def calc_jacobian1(self, g1, g2, j1, j2, x):
        jacobs = np.zeros((len(g1), 4))
        for index,  (g1k, g2k) in enumerate(zip(g1, g2)):
            de1dj1 = (np.cross(np.cross(g1k, j1), g1k)) / (np.linalg.norm(np.cross(g1k, j1)))
            de2dj2 = (np.cross(np.cross(g2k, j2), g2k)) / (np.linalg.norm(np.cross(g2k, j2)))
            dj1dphi = self.calc_deriv_phi(x[0], x[1])
            dj1dtita = self.calc_deriv_tita(x[0], x[1])
            dj2dphi = self.calc_deriv_phi(x[2], x[3])
            dj2dtita = self.calc_deriv_tita(x[2], x[3])
            jacobs[index, 0] = np.matmul(de1dj1, dj1dphi)
            jacobs[index, 1] = np.matmul(de1dj1, dj1dtita)
            jacobs[index, 2] = - np.matmul(de2dj2, dj2dphi)
            jacobs[index, 3] = - np.matmul(de2dj2, dj2dtita)
            
        return jacobs
    
    def calc_jacobian2(self, a1, a2, g1, g2, g1dot, g2dot, o):
        o1 = o[0:3,]
        o2 = o[3:7,]
        jacobs = np.zeros((len(g1), 6))
        for index, (a1k, a2k, g1k, g2k, g1kdot, g2kdot) in \
                        enumerate(zip(a1, a2, g1, g2, g1dot, g2dot)):
            temp = self.calc_gammaT(a1k - self.calc_gamma(o1, g1k, g1kdot), g1k, g1kdot) / np.linalg.norm(a1k -  self.calc_gamma(o1, g1k, g1kdot))
            jacobs[index, 0] = temp[0]
            jacobs[index, 1] = temp[1]
            jacobs[index, 2] = temp[2]

            temp = self.calc_gammaT(a2k - self.calc_gamma(o2, g2k, g2kdot), g2k, g2kdot) / np.linalg.norm(a2k -  self.calc_gamma(o2, g2k, g2kdot))
            jacobs[index, 3] = -temp[0]
            jacobs[index, 4] = -temp[1]
            jacobs[index, 5] = -temp[2]
        return jacobs


    def converge_j1j2(self, g1, g2):
        x = np.array([np.pi/4]*4).T
        xold = np.array([100.0]*4).T
        acceptable = 0.0001
        while not all(abs(x - xold) < acceptable):
            j1, j2 = self.calc_j(x)
            error = self.calc_error1(g1, g2, j1, j2)
            jacobians = self.calc_jacobian1(g1, g2, j1, j2, x)
            xold = x.copy()
            x -= np.matmul(np.linalg.pinv(jacobians), error)
            print(f'the sum of errors is for x is {sum(abs(x - xold))}')
        return self.calc_j(x)

    def calc_gamma(self, oi, gi, gidot):
        return np.cross(gi, np.cross(gi, oi)) + np.cross(gidot, oi)

    def calc_gammaT(self, oi, gi, gidot):
        return np.cross(np.cross(oi, gi), gi) + np.cross(oi, gidot)

    def calc_error2(self, a1, a2, g1, g2, g1dot, g2dot, o):
        o1 = o[0:3]
        o2 = o[3:7]
        error = []
        for a1k, a2k, g1k, g2k, g1kdot, g2kdot in zip(a1, a2, g1, g2, g1dot, g2dot):
            gamma1 = self.calc_gamma(o1, g1k, g1kdot)
            gamma2 = self.calc_gamma(o2, g2k, g2kdot)
            error.append(np.linalg.norm(a1k - gamma1) - np.linalg.norm(a2k - gamma2))
        return error
    
    def converge_o1o2(self, a1, a2, g1, g2, g1dot, g2dot):
        o = np.array([0.0]*6).T
        oold = np.array([1.0]*6).T
        acceptable = 0.001
        while not all(abs(o - oold) < acceptable):
            error = self.calc_error2(a1, a2, g1, g2, g1dot, g2dot, o)
            jacobian = self.calc_jacobian2(a1, a2, g1, g2, g1dot, g2dot, o)
            oold = o.copy()
            o -= np.matmul(np.linalg.pinv(jacobian), np.array(error))
            print(f'the sum of errors is for o is {sum(abs(o - oold))}')
        o1 = o[0:3]
        o2 = o[3:7]
        return o1, o2


    
    def calibrate_pair(self, a1, a2, g1, g2, g1dot, g2dot, t):
        a1, a2, g1, g2, g1dot, g2dot = self.generate_dataset(a1, a2, g1, g2, g1dot, g2dot, t)
        j1, j2 = self.converge_j1j2(g1, g2)
        o1, o2 = self.converge_o1o2(a1, a2, g1, g2, g1dot, g2dot)
        return j1, j2, o1, o2

    
    def calibrate(self,):
        self.gdots = []
        for imu in self.imus:
            self.gdots.append(self.get_gdot(imu.calib_gyro, imu.calib_times))
        j1, j2, o1, o2 = self.calibrate_pair(self.imus[0].calib_accel,
                            self.imus[1].calib_accel,
                            self.imus[0].calib_gyro,
                            self.imus[1].calib_gyro,
                            self.gdots[0],
                            self.gdots[1],
                            self.calib_times)
        self.j1 = j1
        self.j2 = j2
        self.o1 = o1
        self.o2 = o2
        
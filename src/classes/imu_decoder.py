import numpy as np
import glob
import os
from classes.imu import IMU
from scipy.optimize import minimize
import matplotlib.pyplot as plt


class IMUinterpreter:
    def __init__(self, directory_number = -1, file_number = -1):
        current_directory = os.path.dirname(os.path.abspath(__file__))
        # Go up one directory
        parent_directory = os.path.dirname(current_directory)
        # Go into the "data" directory
        data_directory = os.path.join(parent_directory, 'data')
        list_of_directories = [x[0] for x in os.walk(data_directory)if len(x[0])>len(data_directory)]
        list_of_directories.sort()
        self.directory = list_of_directories[directory_number]
        list_of_files = glob.glob(os.path.join(self.directory,'*.bin'))
        list_of_files = [file for file in list_of_files if "calibration" not in file]
        list_of_files.sort()
        calib_filename = os.path.join(self.directory, 'calibration.bin')
        if file_number == None:
            filename = os.path.join(self.directory, 'calibration.bin')
        else:
            filename = list_of_files[file_number]

        if os.path.exists(os.path.join(self.directory, 'calibration.npz')):
            self.calibrated = True
        else:
            self.calibrated = False

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
        # self.clean()
       
        self.times = self.imus[0].times
        self.calib_times = self.imus[0].calib_times

        # for i in range(3):
        #     plt.figure()
        #     plt.plot(self.calib_times, self.imus[i].calib_accel)
        #     plt.show()

    def clean(self,):
        for imu in self.imus:
            imu.clean_all()

    def decode(self, data, calibration = False):
        pointer = 0
        while pointer < len(data)-self.packetsize:
            for imu in self.imus:
                imu.decode_data(data[pointer:(pointer + 12)], calibration)
                pointer += 12
            for imu in self.imus:
                imu.decode_time(data[pointer:(pointer+4)], calibration)
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
        o1 = o[0:3]
        o2 = o[3:7]
        jacobs = np.zeros((len(g1), 6))
        for index, (a1k, a2k, g1k, g2k, g1kdot, g2kdot) in \
                        enumerate(zip(a1, a2, g1, g2, g1dot, g2dot)):
            gammagioi = self.calc_gamma(o1, g1k, g1kdot)
            resta = a1k - gammagioi
            temp = self.calc_gammaT(resta, g1k, g1kdot) / np.linalg.norm(resta)
            jacobs[index, 0] = temp[0]
            jacobs[index, 1] = temp[1]
            jacobs[index, 2] = temp[2]

            gammagioi = self.calc_gamma(o2, g2k, g2kdot)
            resta = a2k - gammagioi
            temp = self.calc_gammaT(resta, g2k, g2kdot) / np.linalg.norm(resta)
            jacobs[index, 3] = -temp[0]
            jacobs[index, 4] = -temp[1]
            jacobs[index, 5] = -temp[2]
        return jacobs

    def calc_error1_abs(self, x, g1, g2):
        j1, j2 = self.calc_j(x)
        
        error = 0
        for g1k, g2k in zip(g1, g2):
            error += (np.linalg.norm(np.cross(g1k, j1)) - np.linalg.norm(np.cross(g2k, j2)))**2
        return error
    
    def converge_j1j2(self, g1, g2):
        # x = np.array([np.pi/4]*4).T
        # xold = np.array([100.0]*4).T
        # acceptable = 0.0001
        # while not all(abs(x - xold) < acceptable):
        #     j1, j2 = self.calc_j(x)
        #     error = self.calc_error1(g1, g2, j1, j2)
        #     jacobians = self.calc_jacobian1(g1, g2, j1, j2, x)
        #     xold = x.copy()
        #     x -= np.matmul(np.linalg.pinv(jacobians), error)
        #     print(f'the sum of errors is for x is {sum(abs(x - xold))}')

        x = np.array([np.pi/4]*4).T
        res = minimize(self.calc_error1_abs, x, method = 'BFGS', args=(g1, g2), options = {'disp':True})
        return self.calc_j(res.x)


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

    def calc_error2_abs(self, o, a1, a2, g1, g2, g1dot, g2dot):
        o1 = o[0:3]
        o2 = o[3:7]
        error = 0
        for a1k, a2k, g1k, g2k, g1kdot, g2kdot in zip(a1, a2, g1, g2, g1dot, g2dot):
            gamma1 = self.calc_gamma(o1, g1k, g1kdot)
            gamma2 = self.calc_gamma(o2, g2k, g2kdot)
            error += (np.linalg.norm(a1k - gamma1) - np.linalg.norm(a2k - gamma2))**2
        return error
    
    def converge_o1o2(self, a1, a2, g1, g2, g1dot, g2dot):
        # o = np.array([0.0]*6).T
        # oold = np.array([0.01]*6).T
        # acceptable = 0.0001
        # while not all(abs(o - oold) < acceptable):
        #     error = self.calc_error2(a1, a2, g1, g2, g1dot, g2dot, o)
        #     jacobian = self.calc_jacobian2(a1, a2, g1, g2, g1dot, g2dot, o)
        #     oold = o.copy()
        #     o += np.matmul(np.linalg.pinv(jacobian), np.array(error))
        #     print(f'the sum of errors is for o is {sum(abs(o - oold))}')
        o = np.array([0.0]*6).T
        res = minimize(self.calc_error2_abs, o, method = 'BFGS', args=(a1, a2, g1, g2, g1dot, g2dot), options= {'disp':True})
        o1 = res.x[0:3]
        o2 = res.x[3:7]
        return o1, o2

    
    def calibrate_pair(self, a1, a2, g1, g2, g1dot, g2dot, t):
        a1, a2, g1, g2, g1dot, g2dot = self.generate_dataset(a1, a2, g1, g2, g1dot, g2dot, t)
        print('Calibrating j1 and j2')
        j1, j2 = self.converge_j1j2(g1, g2)
        print('\nCalibrating o1 and o2')
        o1, o2 = self.converge_o1o2(a1, a2, g1, g2, g1dot, g2dot)
        return j1, j2, o1, o2

    def calibrate(self,):
        if self.calibrated:
            print('Already calibrated!')
            loaded = np.load(os.path.join(self.directory, 'calibration.npz'))
            self.knee_j1 = loaded['knee_j1']
            self.knee_j2 = loaded['knee_j2']
            self.knee_o1 = loaded['knee_o1']
            self.knee_o2 = loaded['knee_o2']
            self.ankle_j1 = loaded['ankle_j1']
            self.ankle_j2 = loaded['ankle_j2']
            self.ankle_o1 = loaded['ankle_o1']
            self.ankle_o2 = loaded['ankle_o2']
            self.gdots = []
            for imu in self.imus:
                self.gdots.append(self.get_gdot(imu.gyro, imu.times))
            print('Data loaded')
        else:
            self.gdots = []
            for imu in self.imus:
                self.gdots.append(self.get_gdot(imu.calib_gyro, imu.calib_times))
            print('Calibrating knee joint vectors')
            j1, j2, o1, o2 = self.calibrate_pair(self.imus[0].calib_accel,
                                self.imus[1].calib_accel,
                                self.imus[0].calib_gyro,
                                self.imus[1].calib_gyro,
                                self.gdots[0],
                                self.gdots[1],
                                self.calib_times)
            signos = ((1,1),(1,-1),(-1,1),(-1,-1))
            plt.figure()
            for signo1, signo2, num in zip([1,1,-1,-1], [1,-1,1,-1], [1, 2, 3, 4]):
                angles = self.get_alpha_gyro(self.imus[0].calib_gyro, self.imus[1].calib_gyro, signo1*j1, signo2*j2)
                plt.plot(np.array(angles)*180/np.pi, label = f'{num}) {signo1} {signo2}')
            plt.legend()
            plt.show()

            signo_1, signo_2 = signos[int(input('Ingrese número seleccionado:    ')) - 1]
                        
            self.knee_j1 = signo_1 * j1
            self.knee_j2 = signo_2 * j2
            self.knee_o1 = o1
            self.knee_o2 = o2
            print('Done calibrating knee')
            print()
            print('Calibrating ankle joint vectors')
            j1, j2, o1, o2 = self.calibrate_pair(self.imus[1].calib_accel,
                                self.imus[2].calib_accel,
                                self.imus[1].calib_gyro,
                                self.imus[2].calib_gyro,
                                self.gdots[1],
                                self.gdots[2],
                                self.calib_times)

            plt.figure()
            for signo1, signo2 , num in zip([1,1,-1,-1], [1,-1,1,-1], [1,2,3,4]):
                angles = self.get_alpha_gyro(self.imus[1].calib_gyro, self.imus[2].calib_gyro, signo1*j1, signo2*j2)
                plt.plot(np.array(angles)*180/np.pi, label = f'{num}) {signo1} {signo2}')
            plt.legend()
            plt.show()

            signo_1, signo_2 = signos[int(input('Ingrese número seleccionado:    ')) - 1]

            self.ankle_j1 = signo_1 * j1
            self.ankle_j2 = signo_2 * j2
            self.ankle_o1 = o1
            self.ankle_o2 = o2
            print('Done calibrating ankle')
            print()
            print('Saving files')
            np.savez(os.path.join(self.directory, 'calibration.npz'),
                    knee_j1 = self.knee_j1,
                    knee_j2 = self.knee_j2,
                    knee_o1 = self.knee_o1,
                    knee_o2 = self.knee_o2,
                    ankle_j1 = self.ankle_j1,
                    ankle_j2 = self.ankle_j2,
                    ankle_o1 = self.ankle_o1,
                    ankle_o2 = self.ankle_o2)
            self.gdots = []
            for imu in self.imus:
                self.gdots.append(self.get_gdot(imu.gyro, imu.times))
    
    def signed_angle(self, v1, v2):
        """
        Calculate the signed angle between two 2D vectors.
        
        Parameters:
        v1 (array-like): The first vector.
        v2 (array-like): The second vector.
        
        Returns:
        float: The signed angle between v1 and v2 in radians.
        """
        # Convert to numpy arrays
        v1 = np.array(v1)
        v2 = np.array(v2)
        
        # Calculate the angle between the vectors
        angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
        
        # Normalize the angle to the range [-π, π]
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        
        return angle

    def get_alpha_gyro(self, g1, g2, j1, j2):
        dt = np.average(np.diff(self.times))
        angle = []
        for g1k, g2k in zip(g1, g2):
            if angle == []:
                angle.append((np.matmul(g1k, j1) - np.matmul(g2k,j2))*dt)
            else:
                angle.append(angle[-1] + (np.matmul(g1k, j1) - np.matmul(g2k, j2))*dt)
        return angle
    
    def get_alpha_accel(self, o1, o2, j1, j2, g1, g2, a1, a2, g1dot, g2dot):
        o1rot = o1 - j1 * (np.dot(o1, j1) + np.dot(o2, j2)) * 0.5
        o2rot = o2 - j2 * (np.dot(o1, j1) + np.dot(o2, j2)) * 0.5
        
        x1 = np.cross(j1, [1,1,1])
        y1 = np.cross(j1, x1)

        x2 = np.cross(j2, [1,1,1])
        y2 = np.cross(j2, x2)

        angle = []
        supp = 0
        # flag = True
        for a1k, a2k, g1k, g2k, g1kdot, g2kdot in zip(a1[2:-2],
                                                      a2[2:-2],
                                                      g1[2:-2],
                                                      g2[2:-2],
                                                      g1dot, g2dot):
            a1prime = a1k - self.calc_gamma(o1rot, g1k, g1kdot)
            a2prime = a2k - self.calc_gamma(o2rot, g2k, g2kdot)

            temp1 = np.array([np.dot(a1prime, x1), np.dot(a1prime, y1)])
            temp2 = np.array([np.dot(a2prime, x2), np.dot(a2prime, y2)])

            current = self.signed_angle(temp1, temp2)
            # if flag:
            #     flag = False
            #     past = current
            # if current - past > np.pi:
            #     supp -= 2*np.pi
            # elif past - current > np.pi:
            #     supp += 2*np.pi


            # past = current
            angle.append(current + supp)
        
        return angle

    def get_alpha_fusion(self, gyro, accel):
        gyro = np.array(gyro)+accel[0]
        angle = [accel[0]]
        lamb = 0.015
        for ag, aa, agold in zip(gyro[3:-2], accel[1:], gyro[2:-3]):
            angle.append(lamb*aa + (1 -lamb)*(angle[-1] + ag - agold))
        
        return np.array(gyro), np.array(accel), np.array(angle)
    
    def get_data(self, joint):
        if joint == 'knee':
            return self.knee_o1, self.knee_o2, self.knee_j1, self.knee_j2, \
                   self.imus[0].gyro, self.imus[1].gyro, \
                   self.imus[0].accel, self.imus[1].accel, \
                   self.gdots[0], self.gdots[1]
        elif joint == 'ankle':
            return self.ankle_o1, self.ankle_o2, self.ankle_j1, self.ankle_j2, \
                   self.imus[1].gyro, self.imus[2].gyro, \
                   self.imus[1].accel, self.imus[2].accel, \
                   self.gdots[1], self.gdots[2]
    
    def calculate_angles(self, ):
        for i in ['knee', 'ankle']:
            o1, o2, j1, j2, g1, g2, a1, a2, g1dot, g2dot = self.get_data(i)
            gyro = self.get_alpha_gyro(g1, g2, j1, j2)
            accel = self.get_alpha_accel(o1, o2, j1, j2, g1, g2, a1, a2, g1dot, g2dot)
            if i == 'knee':
                self.knee_gyro, self.knee_accel, self.knee_fusion = self.get_alpha_fusion(gyro, accel)
            else:
                self.ankle_gyro, self.ankle_accel, self.ankle_fusion = self.get_alpha_fusion(gyro, accel)



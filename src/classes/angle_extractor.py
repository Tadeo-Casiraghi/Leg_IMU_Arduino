import numpy as np
import glob
import os
from scipy.optimize import minimize
from scipy.signal import find_peaks, butter, filtfilt
import matplotlib.pyplot as plt
import scipy.io as sio
import pandas as pd
from classes.imu_decoder import read_data

graph = False

class ANGLE_extractor:
    def __init__(self, directory_name = 'data', sub_directory_name = -1, file_name = -1):
        current_directory = os.path.dirname(os.path.abspath(__file__))
        # Go up one directory
        parent_directory = os.path.dirname(current_directory)
        data_directory = os.path.join(parent_directory, directory_name)
        list_of_directories = [x[0] for x in os.walk(data_directory)if len(x[0])>len(data_directory)]
        list_of_directories.sort()

        if type(sub_directory_name) == int:
            self.directory = list_of_directories[sub_directory_name]
        else:
            self.directory = os.path.join(data_directory, sub_directory_name)
        
        self.settings = pd.read_json(os.path.join(data_directory, 'settings.json'), typ='series')
        
        list_of_files = glob.glob(os.path.join(self.directory,'*'))
        list_of_files.sort()
        if any(".bin" in file for file in list_of_files):
            list_of_files_bin = [file for file in list_of_files if ".bin" in file]
            list_of_files_csv = [file for file in list_of_files if ".csv" in file]
            if type(file_name) == int:
                filename = list_of_files_bin[file_name]
                if filename[:-4] + '.csv' in list_of_files_csv:
                    print('File already converted')
                else:
                    read_data(filename)
                filename = filename[:-4] + '.csv'
            else:
                if os.path.join(self.directory, filename[:-4] + '.csv') in list_of_files_csv:
                    print('File already converted')
                else:
                    read_data(os.path.join(self.directory, filename))
                filename = os.path.join(self.directory, filename[:-4] + '.csv')
        else:
            if type(file_name) == int:
                filename = list_of_files[file_name]
            else:
                filename = os.path.join(self.directory, file_name)
        
        self.imus = {'header' : [],
                     'foot': {'accel': [], 'gyro': []},
                     'shank': {'accel': [], 'gyro': []},
                     'thigh': {'accel': [], 'gyro': []}}
        
        self.peaked = False

        data = pd.read_csv(filename)
        
        self.raw_accel = self.settings['raw_accel']
        self.raw_gyro = self.settings['raw_gyro']

        for _, row in data.iterrows():
            self.imus['header'].append(row[self.settings['names']['Header']])
            self.imus['foot']['accel'].append(np.array([row[self.settings['names']['foot_Accel_X']],
                                                        row[self.settings['names']['foot_Accel_Y']],
                                                        row[self.settings['names']['foot_Accel_Z']]])
                                                        *self.settings['scale_accel'])
            
            self.imus['foot']['gyro'].append(np.array([row[self.settings['names']['foot_Gyro_X']],
                                                        row[self.settings['names']['foot_Gyro_Y']],
                                                        row[self.settings['names']['foot_Gyro_Z']]])
                                                        *self.settings['scale_gyro'])
            
            self.imus['shank']['accel'].append(np.array([row[self.settings['names']['shank_Accel_X']],
                                                        row[self.settings['names']['shank_Accel_Y']],
                                                        row[self.settings['names']['shank_Accel_Z']]])
                                                        *self.settings['scale_accel'])
            
            self.imus['shank']['gyro'].append(np.array([row[self.settings['names']['shank_Gyro_X']],    
                                                        row[self.settings['names']['shank_Gyro_Y']],
                                                        row[self.settings['names']['shank_Gyro_Z']]])
                                                        *self.settings['scale_gyro'])
            
            self.imus['thigh']['accel'].append(np.array([row[self.settings['names']['thigh_Accel_X']],
                                                        row[self.settings['names']['thigh_Accel_Y']],
                                                        row[self.settings['names']['thigh_Accel_Z']]])
                                                        *self.settings['scale_accel'])
            
            self.imus['thigh']['gyro'].append(np.array([row[self.settings['names']['thigh_Gyro_X']],
                                                        row[self.settings['names']['thigh_Gyro_Y']],
                                                        row[self.settings['names']['thigh_Gyro_Z']]])
                                                        *self.settings['scale_gyro'])
            
        if graph:
            counter = 1
            for key, value in self.imus.items():
                if key == 'header':
                    continue
                for key2, value2 in value.items():
                    plt.subplot(3,2,counter)
                    plt.plot(value2)
                    plt.title(f'{key} {key2}')
                    counter += 1
            plt.show()

        if self.settings['ground_truth']['available']:
            ground_truth = pd.read_csv(os.path.join(self.directory, self.settings['ground_truth']['path']))
            self.ground = {'header': ground_truth[self.settings['ground_truth']['header']],
                           'knee': ground_truth[self.settings['ground_truth']['knee']],
                           'ankle': ground_truth[self.settings['ground_truth']['ankle']]}
        else:
            self.ground = None
            
        if os.path.exists(os.path.join(self.directory, 'calibration.npz')):
            self.calibrated = True
        else:
            self.calibrated = False
        

    def get_gdot(self, g, t):
        dt = np.average(np.diff(t))
        gdot = []
        for g0, g1, g2, g3 in zip(g[:-4], g[1:-3], g[3:-1], g[4:]):
            gdot.append((g0 - 8*g1 + 8*g2 - g3)/(12*dt))
        return np.array(gdot)


    def find_median(self, numbers):
        # Sort the list
        sorted_numbers = sorted(numbers)
        n = len(sorted_numbers)
        
        # If the list length is odd, return the middle element
        if n % 2 == 1:
            return sorted_numbers[n // 2]
        # If the list length is even, return the average of the two middle elements
        else:
            middle1 = sorted_numbers[n // 2 - 1]
            middle2 = sorted_numbers[n // 2]
            return (middle1 + middle2) / 2
    

    def get_peaks(self):
        self.peaked = True
        values = self.imus['foot']['accel']
        maxi = [0,0,0]
        mini = [10000]*3
        for value in values:
            for i in range(3):
                maxi[i] = max(maxi[i], abs(value[i]))
                mini[i] = min(mini[i], abs(value[i]))

        pos = np.argmax([maxi[i] - mini[i] for i in range(3)])
        accel = []
        for value in values:
            accel.append(value[pos])

        averg = self.find_median(accel)      

        if (max(accel) - averg) < (mini[pos] - averg):
            accel = [-x for x in accel]
        
        times = self.imus['header']

        # plt.figure()
        # plt.plot(times, accel)
        # plt.show()

        # ranges = input('Enter ranges for peaks:    ')

        # ranges = ranges.split(' ')
        # ranges = [float(x) for x in ranges]

        starter = 0
        ender = len(times)-1
        # for t in times:
        #     if t < ranges[0]:
        #         starter += 1
        #     if t < ranges[1]:
        #         ender += 1
        #     else:
        #         break
        
        acce_temp = accel[starter:ender]
        val_range = maxi[pos] - mini[pos]
        peaks = find_peaks(acce_temp, prominence = val_range/6, distance = (1/np.average(np.diff(times)))/2)
        
        peaks = [pk + starter for pk in peaks[0]]
        
        # plt.figure()
        # plt.plot(times, accel)
        # x, y = [], []
        # for peak in peaks:
        #     x.append(times[peak])
        #     y.append(accel[peak])
        # plt.scatter(x, y, color = 'red')
        # plt.show()

        self.peaks = peaks

        timesets = []
        timesets2 = []

        for index0, index1 in zip(peaks[:-1], peaks[1:]):
            p40 = times[index0] + 0.4*(times[index1] - times[index0])
            p80 = times[index0] + 0.8*(times[index1] - times[index0])
            timesets.append((p40, p80))
            p0 = times[index0]
            p100 = times[index1]
            timesets2.append((p0, p100))

        self.timesets = timesets
        self.timesets2 = timesets2


    def generate_dataset(self, data):
        '''
        data in the form data = {'header': [times], 'a1': [data], 'a2': [data],
                                                    'g1': [data], 'g2': [data],
                                                    'gd1': [data], 'gd2': [data]}
        '''

        S = []

        for key in data.keys():
            if key == 'header':
                continue
            S.append([])
        
        group = 0
        flip_on = False
        told = data['header'][0]
        T = 0.1
        
        t = data['header']

        for index, tt in enumerate(t):
            if group == len(self.timesets):
                break
            if not flip_on and tt > self.timesets[group][0]:
                flip_on = True
            elif flip_on and tt > self.timesets[group][1]:
                flip_on = False
                group += 1

            if flip_on and (tt - told) > T:
                told = tt
                for id, (key, value) in enumerate(data.items()):
                    if key == 'header':
                        continue

                    S[id-1].append(value[index])
        return S


    def calc_j(self, x):
        j1 = np.array([np.cos(x[0])*np.cos(x[1]),
                       np.cos(x[0])*np.sin(x[1]),
                       np.sin(x[0])]).T
        j2 = np.array([np.cos(x[2])*np.cos(x[3]),
                       np.cos(x[2])*np.sin(x[3]),
                       np.sin(x[2])]).T
        return j1, j2
    
    
    def calc_error1_abs(self, x, g1, g2):
        j1, j2 = self.calc_j(x)
        
        error = 0
        for g1k, g2k in zip(g1, g2):
            error += (np.linalg.norm(np.cross(g1k, j1)) - np.linalg.norm(np.cross(g2k, j2)))**2
        return error
    
    
    def converge_j1j2(self, g1, g2):
        x = np.array([0]*4).T
        res = minimize(self.calc_error1_abs, x, method = 'BFGS', args=(g1, g2), options = {'disp':True})
        return self.calc_j(res.x)


    def calc_gamma(self, oi, gi, gidot):
        return np.cross(gi, np.cross(gi, oi)) + np.cross(gidot, oi)


    def calc_error2_abs(self, o, a1, a2, g1, g2, g1dot, g2dot):
        o1 = o[0:3]
        o2 = o[3:6]
        error = 0
        for a1k, a2k, g1k, g2k, g1kdot, g2kdot in zip(a1, a2, g1, g2, g1dot, g2dot):
            gamma1 = self.calc_gamma(o1, g1k, g1kdot)
            gamma2 = self.calc_gamma(o2, g2k, g2kdot)
            error += (np.linalg.norm(a1k - gamma1) - np.linalg.norm(a2k - gamma2))**2
        return error
    

    def converge_o1o2(self, a1, a2, g1, g2, g1dot, g2dot):
        o = np.array([0.0]*6).T
        res = minimize(self.calc_error2_abs, o, method = 'BFGS', args=(a1, a2, g1, g2, g1dot, g2dot), options= {'disp':True})
        o1 = res.x[0:3]
        o2 = res.x[3:6]
        return o1, o2


    def apply_filter(self, b, a, signal):
        x, y, z = [], [], []
        for (X, Y, Z) in signal:
            x.append(X)
            y.append(Y)
            z.append(Z)
        
        xf = filtfilt(b, a, x)
        yf = filtfilt(b, a, y)
        zf = filtfilt(b, a, z)


        output = []
        for X, Y, Z in zip(xf, yf, zf):
            output.append(np.array([X, Y, Z]))
        return output
    

    def calibrate_pair(self, a1, a2, g1, g2, t):
        g1ds, g2ds = self.generate_dataset({'header':t, 'g1':g1, 'g2':g2})
        print('Calibrating j1 and j2')
        j1, j2 = self.converge_j1j2(g1ds, g2ds)

        b, a = butter(2, 7, fs = 1/np.average(np.diff(t)))
        g1f = self.apply_filter(b, a, g1)
        g2f = self.apply_filter(b, a, g2)


        g1dot = self.get_gdot(g1f, t)
        g2dot = self.get_gdot(g2f, t)
        
        data = {'header': t[2:], 'a1': a1[2:], 'a2': a2[2:], 'g1': g1f[2:], 'g2': g2f[2:], 'g1dot': g1dot, 'g2dot': g2dot}

        a1fds, a2fds, g1fds, g2fds, g1dotfds, g2dotfds = self.generate_dataset(data)
        print('\nCalibrating o1 and o2')
        o1, o2 = self.converge_o1o2(a1fds, a2fds, g1fds, g2fds, g1dotfds, g2dotfds)
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
            print('Data loaded')
        else:
            self.get_peaks()
            print('Calibrating knee joint vectors')
            j1, j2, o1, o2 = self.calibrate_pair(self.imus['thigh']['accel'],
                                                 self.imus['shank']['accel'],
                                                 self.imus['thigh']['gyro'],
                                                 self.imus['shank']['gyro'],
                                                 self.imus['header'])
            
            signos = ((1,1),(1,-1),(-1,1),(-1,-1))
            plt.figure()

            for signo1, signo2, num in zip([1,1,-1,-1], [1,-1,1,-1], [1, 2, 3, 4]):
                angles = self.get_alpha_gyro(self.imus['thigh']['gyro'],
                                             self.imus['shank']['gyro'],
                                             signo1*j1, signo2*j2)
                
                plt.plot(self.imus['header'], np.array(angles)*180/np.pi, label = f'{num}) {signo1} {signo2}')
            if self.ground != None:
                plt.plot(self.ground['header'], self.ground['knee'], label = 'Ground truth', color = 'black')
            plt.legend()
            plt.show()

            signo_1, signo_2 = signos[int(input('Ingrese número seleccionado:    ')) - 1]
                        
            self.knee_j1 = signo_1 * j1
            self.knee_j2 = signo_2 * j2
            self.knee_o1 = o1 - self.knee_j1 * (np.dot(o1, self.knee_j1) + np.dot(o2, self.knee_j2)) * 0.5
            self.knee_o2 = o2 - self.knee_j2 * (np.dot(o1, self.knee_j1) + np.dot(o2, self.knee_j2)) * 0.5
            print('Done calibrating knee')
            print()
            print('Calibrating ankle joint vectors')
            j1, j2, o1, o2 = self.calibrate_pair(self.imus['shank']['accel'],
                                                 self.imus['foot']['accel'],
                                                 self.imus['shank']['gyro'],
                                                 self.imus['foot']['gyro'],
                                                 self.imus['header'])

            plt.figure()
            for signo1, signo2 , num in zip([1,1,-1,-1], [1,-1,1,-1], [1,2,3,4]):
                angles = self.get_alpha_gyro(self.imus['shank']['gyro'],
                                             self.imus['foot']['gyro'],
                                             signo1*j1, signo2*j2)
                plt.plot(self.imus['header'], np.array(angles)*180/np.pi, label = f'{num}) {signo1} {signo2}')
            if self.ground != None:
                plt.plot(self.ground['header'], self.ground['ankle'], label = 'Ground truth', color = 'black')
            plt.legend()
            plt.show()

            signo_1, signo_2 = signos[int(input('Ingrese número seleccionado:    ')) - 1]

            self.ankle_j1 = signo_1 * j1
            self.ankle_j2 = signo_2 * j2
            self.ankle_o1 = o1 - self.ankle_j1 * (np.dot(o1, self.ankle_j1) + np.dot(o2, self.ankle_j2)) * 0.5
            self.ankle_o2 = o2 - self.ankle_j2 * (np.dot(o1, self.ankle_j1) + np.dot(o2, self.ankle_j2)) * 0.5
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
    

    def get_alpha_gyro(self, g1, g2, j1, j2):
        dt = np.average(np.average(np.diff(self.imus['header'])))
        angle = []
        b, a = butter(2, 7, fs = 1/dt)
        print(1/dt)
        g1f = self.apply_filter(b, a, g1)
        g2f = self.apply_filter(b, a, g2)
        for g1k, g2k in zip(g1f, g2f):
            if angle == []:
                angle.append((np.matmul(g1k, j1) - np.matmul(g2k,j2))*dt)
            else:
                angle.append(angle[-1] + (np.matmul(g1k, j1) - np.matmul(g2k,j2))*dt)
        return angle
    

    def get_alpha_accel(self, o1, o2, j1, j2, g1, g2, a1, a2, joint):
        dt = np.average(np.diff(self.imus['header']))
        b, a = butter(2, 7, fs = 1/dt)
        g1f = self.apply_filter(b, a, g1)
        g2f = self.apply_filter(b, a, g2)
        a1f = self.apply_filter(b, a, a1)
        a2f = self.apply_filter(b, a, a2)
        g1dot = self.get_gdot(g1f, self.imus['header'])
        g2dot = self.get_gdot(g2f, self.imus['header'])

        x1 = np.cross(j1, [1,0,0])
        y1 = np.cross(j1, x1)

        x2 = np.cross(j2, [1,0,0])
        y2 = np.cross(j2, x2)

        angle = []
        for a1k, a2k, g1k, g2k, g1kdot, g2kdot in zip(a1f[2:-2],
                                                      a2f[2:-2],
                                                      g1f[2:-2],
                                                      g2f[2:-2],
                                                      g1dot, g2dot):
            a1prime = a1k - self.calc_gamma(o1, g1k, g1kdot)
            a2prime = a2k - self.calc_gamma(o2, g2k, g2kdot)

            temp1 = np.array([np.dot(a1prime, x1), np.dot(a1prime, y1)])
            temp2 = np.array([np.dot(a2prime, x2), np.dot(a2prime, y2)])
            
            
            current = self.signed_angle(temp1, temp2)
            if joint == 'knee':
                current = -current
            angle.append(current)

        # if joint == 'ankle':
        #     maxi = 0
        #     mini = -4
        # else:
        #     maxi = 2
        #     mini = -4

        # for current in angle:
        #     if current < mini:
        #         angle2.append(current + 2*np.pi)
        #     elif current > maxi:
        #         angle2.append(current - 2*np.pi)
        #     else:
        #         angle2.append(current)
        #     angle2[-1] = -angle2[-1]
        
        # derivada = [a - b for a, b in zip(angle[1:], angle[:-1])]
        # # angle2 = [angle[0]]
        # calib = 0
        # for drv, ang in zip(derivada, angle[1:]):
        #     if drv > 5.6:
        #         calib += 2*np.pi
        #     elif drv < -5.6:
        #         calib -= 2*np.pi
        #     angle2.append(ang - calib)

        # fig, ax1 = plt.subplots()

        # ax1.plot(self.imus['header'][2:-2], angle, color = 'black', label = 'Estimated angle')
        # # ax1.plot(self.imus['header'][2:-2], angle2, color = 'blue', label = 'Estimated angle 2')
        # if self.ground != None:
        #     ax1.plot(self.ground['header'], self.ground['knee']/180*np.pi, label = 'Ground truth', color = 'red')
        # ax1.legend()

        # ax2 = ax1.twinx()
        # ax2.plot(self.imus['header'][3:-2], derivada, color = 'green', label = 'Derivative')
        # plt.show()
        return angle

    
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
        
    
    def get_alpha_fusion(self, gyro, accel, lamb = 0.0005):
        gyro = np.array(gyro)+accel[0]
        angle = [accel[0]]
        group = 0
        flag = False
        new_times = [self.imus['header'][2]]
        for ag, aa, agold, t in zip(gyro[3:-2], accel[1:], gyro[2:-3], self.imus['header'][3:]):
            new_times.append(t)
            if group == len(self.timesets2):
                angle.append(lamb*aa + (1 -lamb)*(angle[-1] + ag - agold))
                continue
            if not flag and t > self.timesets2[group][0]:
                flag = True
            elif flag and t > self.timesets2[group][0] + 0.05 * (self.timesets2[group][1] - self.timesets2[group][0]):
                flag = False

            if t > self.timesets2[group][0] + 0.95 * (self.timesets2[group][1] - self.timesets2[group][0]):
                flag = True
                group += 1

            if flag:
                angle.append(angle[-1] + ag - agold)
            else:
                angle.append(lamb*aa + (1 -lamb)*(angle[-1] + ag - agold))
        
        return np.array(gyro), np.array(accel), np.array(angle), np.array(new_times)
    
    def get_data(self, joint):
        if joint == 'knee':
            return self.knee_o1, self.knee_o2, self.knee_j1, self.knee_j2, \
                   self.imus['thigh']['gyro'], self.imus['shank']['gyro'], \
                   self.imus['thigh']['accel'], self.imus['shank']['accel']
        elif joint == 'ankle':
            return self.ankle_o1, self.ankle_o2, self.ankle_j1, self.ankle_j2, \
                   self.imus['shank']['gyro'], self.imus['foot']['gyro'], \
                   self.imus['shank']['accel'], self.imus['foot']['accel']
    

    def lambda_search(self, vars, joint, gyro, accel):
        lamb = vars[0]
        if joint == 'knee':
            self.knee_gyro, self.knee_accel, self.knee_fusion, self.knee_time = self.get_alpha_fusion(gyro, accel, lamb)
            temp = (self.knee_fusion)*180/np.pi
            error_current = np.sum([((a - b)**2)/1000000 for a, b in zip(temp, self.ground['knee'])])
        else:
            self.ankle_gyro, self.ankle_accel, self.ankle_fusion, self.ankle_time = self.get_alpha_fusion(gyro, accel, lamb)
            temp = (self.ankle_fusion)*180/np.pi
            error_current = np.sum([((a - b)**2)/1000000 for a, b in zip(temp, self.ground['ankle'])])
        return error_current


    def calculate_angles(self, joint, lamb = 0.0005):
        if self.peaked == False:
            self.get_peaks()
        
        o1, o2, j1, j2, g1, g2, a1, a2  = self.get_data(joint)
        gyro = self.get_alpha_gyro(g1, g2, j1, j2)
        accel = self.get_alpha_accel(o1, o2, j1, j2, g1, g2, a1, a2, joint)
        if type(lamb) != list:   
            if joint == 'knee':
                self.knee_gyro, self.knee_accel, self.knee_fusion, self.knee_time = self.get_alpha_fusion(gyro, accel, lamb)
            else:
                self.ankle_gyro, self.ankle_accel, self.ankle_fusion, self.ankle_time = self.get_alpha_fusion(gyro, accel, lamb)
        
        else:
            lamb = 0
            res = minimize(self.lambda_search, [lamb], method='Nelder-Mead', args=(joint, gyro, accel), options= {'disp':True})
            

            print(f'Lambda selected for {joint}: {res.x[0]}')

            if joint == 'knee':
                self.knee_gyro, self.knee_accel, self.knee_fusion, self.knee_time = self.get_alpha_fusion(gyro, accel, res.x[0])
            else:
                self.ankle_gyro, self.ankle_accel, self.ankle_fusion, self.ankle_time = self.get_alpha_fusion(gyro, accel, res.x[0])

from classes.angle_extractor import ANGLE_extractor
import matplotlib.pyplot as plt
import numpy as np
import argparse
from numpy.fft import fft, ifft

parser = argparse.ArgumentParser(description='Extract angles from IMU data')
parser.add_argument("-dn", "--directoryname", help='Name of directory containing IMU data', default = 'dataset')
parser.add_argument('-sdn', "--subdirname", help='Name of subdirectory containing IMU data', default = "test1")
parser.add_argument('-fn', "--filename", help='Name of file containing IMU data', default = "imus.csv")

args = parser.parse_args()

directory_name = args.directoryname
sub_directory_name = args.subdirname if any(value.isalpha() for value in args.subdirname) else int(args.subdirname)
file_name = args.filename if any(value.isalpha() for value in args.filename) else int(args.filename)

angles = ANGLE_extractor(directory_name, sub_directory_name, file_name)

angles.calibrate()

angles.calculate_angles('knee', lamb = 0.05)

angles.calculate_angles('ankle', lamb = 0.05)

angles.save_angles()

plt.figure()
plt.plot(angles.imus['header'], angles.knee_gyro*180/np.pi, label = 'gyro', alpha=0.2)
plt.plot(angles.imus['header'][2:-2], angles.knee_accel*180/np.pi, label = 'accel', alpha=0.2)
plt.plot(angles.knee_time, angles.knee_fusion*180/np.pi, label = 'accel + gyro')
plt.legend()
plt.title('Knee')


plt.figure()
plt.plot(angles.imus['header'], angles.ankle_gyro*180/np.pi, label = 'gyro', alpha=0.2)
plt.plot(angles.imus['header'][2:-2], angles.ankle_accel*180/np.pi, label = 'accel', alpha=0.2)
plt.plot(angles.ankle_time, angles.ankle_fusion*180/np.pi, label = 'accel + gyro')
plt.legend()
plt.title('Ankle')

if angles.ground != None:
    plt.figure()
    plt.plot(angles.knee_time, angles.knee_fusion*180/np.pi, label = 'accel + gyro')
    plt.plot(angles.ground['header'], angles.ground['knee'], label = 'ground truth')
    plt.legend()
    plt.title('Knee')
    error_knee = angles.knee_fusion*180/np.pi - angles.ground['knee'][2:-2]
    plt.figure()
    plt.plot(angles.knee_time, error_knee)
    plt.title('Knee Error')

    error_ankle = angles.ankle_fusion*180/np.pi - angles.ground['ankle'][2:-2]
    plt.figure()
    plt.plot(angles.ankle_time, error_ankle)
    plt.title('Ankle Error')

    plt.figure()
    plt.plot(angles.ankle_time, angles.ankle_fusion*180/np.pi, label = 'accel + gyro')
    plt.plot(angles.ground['header'], angles.ground['ankle'], label = 'ground truth')
    plt.legend()
    plt.title('Ankle')

    # get rms error
    knee_error = np.sqrt(np.mean(error_knee**2))
    print('Knee RMS Error: ', knee_error)

    ankle_error = np.sqrt(np.mean(error_ankle**2))
    print('Ankle RMS Error: ', ankle_error)

    sampling_rate = 200
    T = 1.0/sampling_rate
    time = angles.knee_time

    fft_output = fft(error_knee)
    fft_magnitude = np.abs(fft_output)
    fft_freq = np.fft.fftfreq(len(fft_output), T)

    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(time, error_knee)
    plt.title('Error Signal')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (degrees)')

    plt.subplot(2,1,2)
    plt.plot(fft_freq[:len(fft_freq)//2], fft_magnitude[:len(fft_magnitude)//2])
    plt.title('FFT of Error Signal')
    plt.xlabel('Frequency (Hz)')   
    plt.ylabel('Magnitude')

    plt.tight_layout()

plt.show()
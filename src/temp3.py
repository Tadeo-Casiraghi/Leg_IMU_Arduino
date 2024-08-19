from classes.imu_decoder import IMUinterpreter
import matplotlib.pyplot as plt
import numpy as np

decoder = IMUinterpreter()


decoder.calibrate()

angle = []

dt = np.average(np.diff(decoder.calib_times))

for g1k, g2k in zip(decoder.imus[0].calib_gyro, decoder.imus[1].calib_gyro):
    if angle == []:
        angle.append((np.matmul(g1k, decoder.j1) - np.matmul(g2k, decoder.j2))*dt)
    else:
        angle.append(angle[-1] + (np.matmul(g1k, decoder.j1) - np.matmul(g2k, decoder.j2))*dt)

plt.plot(decoder.calib_times, angle)
plt.show()
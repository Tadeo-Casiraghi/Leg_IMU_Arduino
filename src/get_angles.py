from classes.imu_decoder import IMUinterpreter
import matplotlib.pyplot as plt
import numpy as np



decoder = IMUinterpreter(file_number=1)

decoder.calibrate()

decoder.calculate_angles()

plt.figure()
plt.plot(decoder.times, decoder.knee_gyro*180/np.pi, label = 'gyro')
plt.plot(decoder.times[2:-2], decoder.knee_accel*180/np.pi, label = 'accel')
plt.plot(decoder.times[2:-2], decoder.knee_fusion*180/np.pi, label = 'accel + gyro')
plt.legend()
plt.title('Knee')


plt.figure()
plt.plot(decoder.times, decoder.ankle_gyro*180/np.pi, label = 'gyro')
plt.plot(decoder.times[2:-2], decoder.ankle_accel*180/np.pi, label = 'accel')
plt.plot(decoder.times[2:-2], decoder.ankle_fusion*180/np.pi, label = 'accel + gyro')
plt.legend()
plt.title('Ankle')
plt.show()
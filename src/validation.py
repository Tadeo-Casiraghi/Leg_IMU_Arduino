from classes.imu_decoder import IMUinterpreter
import matplotlib.pyplot as plt
import numpy as np

decoder = IMUinterpreter(dataset=True)
decoder.calibrate()
decoder.calculate_angles()

print(decoder.knee_o1)
print(decoder.knee_o2)
print(decoder.ankle_o1)
print(decoder.ankle_o2)


plt.figure()
# plt.plot(decoder.times, decoder.knee_gyro*180/np.pi, label = 'gyro', alpha=0.2)
# plt.plot(decoder.times[2:-2], decoder.knee_accel*180/np.pi, label = 'accel', alpha=0.2)
plt.plot(decoder.knee_time, decoder.knee_fusion*180/np.pi-210, label = 'accel + gyro')
plt.plot(decoder.ground_times, decoder.ground_knee_r, label = 'ground truth')
plt.legend()
plt.title('Knee')


plt.figure()
# plt.plot(decoder.times, decoder.ankle_gyro*180/np.pi, label = 'gyro', alpha=0.2)
# plt.plot(decoder.times[2:-2], decoder.ankle_accel*180/np.pi, label = 'accel', alpha=0.2)
plt.plot(decoder.ankle_time, decoder.ankle_fusion*180/np.pi, label = 'accel + gyro')
plt.plot(decoder.ground_times, decoder.ground_ankle_r, label = 'ground truth')
plt.legend()
plt.title('Ankle')
plt.show()
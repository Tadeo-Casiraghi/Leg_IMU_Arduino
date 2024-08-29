from classes.imu_decoder import IMUinterpreter
import matplotlib.pyplot as plt
import numpy as np


decoder = IMUinterpreter(directory_number=-1,file_number=None)

decoder.calibrate()

decoder.calculate_angles()

print(decoder.knee_o1)
print(decoder.knee_o2)
print(decoder.ankle_o1)
print(decoder.ankle_o2)

# for i in range(3):
#     plt.figure()
#     plt.plot(decoder.imus[i].times, decoder.imus[i].accel, label = 'accel')
#     plt.title(i)
#     plt.legend()
#     plt.figure()
#     plt.plot(decoder.imus[i].times, decoder.imus[i].gyro, label = 'gyro')
#     plt.legend()
#     plt.title(i)

# plt.show()



plt.figure()
plt.plot(decoder.times, decoder.knee_gyro*180/np.pi, label = 'gyro', alpha=0.2)
plt.plot(decoder.times[2:-2], decoder.knee_accel*180/np.pi, label = 'accel', alpha=0.2)
plt.plot(decoder.times[2:-2], decoder.knee_fusion*180/np.pi, label = 'accel + gyro')
plt.legend()
plt.title('Knee')


plt.figure()
plt.plot(decoder.times, decoder.ankle_gyro*180/np.pi, label = 'gyro', alpha=0.2)
plt.plot(decoder.times[2:-2], decoder.ankle_accel*180/np.pi, label = 'accel', alpha=0.2)
plt.plot(decoder.times[2:-2], decoder.ankle_fusion*180/np.pi, label = 'accel + gyro')
plt.legend()
plt.title('Ankle')
plt.show()
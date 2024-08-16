from classes.imu_decoder import IMUinterpreter
import matplotlib.pyplot as plt

# def quaternion_from_vectors(v1, v2):
#     """
#     Find the quaternion that rotates v1 to v2.
#     :param v1: A 3-element array representing the first vector.
#     :param v2: A 3-element array representing the second vector.
#     :return: A 4-element array representing the quaternion (w, x, y, z).
#     """
#     # Normalize the vectors
#     v1 = v1 / np.linalg.norm(v1)
#     v2 = v2 / np.linalg.norm(v2)
    
#     # Compute the cross product and dot product
#     cross_prod = np.cross(v1, v2)
#     dot_prod = np.dot(v1, v2)
    
#     # Compute the scalar part of the quaternion
#     w = np.sqrt((np.linalg.norm(v1) ** 2) * (np.linalg.norm(v2) ** 2)) + dot_prod
    
#     # Compute the vector part of the quaternion
#     xyz = cross_prod
    
#     # Combine to form the quaternion
#     quaternion = np.array([w, xyz[0], xyz[1], xyz[2]])
    
#     # Normalize the quaternion
#     quaternion = quaternion / np.linalg.norm(quaternion)
    
#     return quaternion

# def quaternion_to_rotation_matrix(q):
#     """
#     Convert a quaternion into a rotation matrix.
#     Quaternion is in the form [w, x, y, z].
#     """
#     w, x, y, z = q
#     return np.array([
#         [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w    ],
#         [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w    ],
#         [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
#     ])

# def rotate_vector_by_quaternion(q, v):
#     """
#     Rotate a vector v by the quaternion q.
#     q is in the form [w, x, y, z], and v is in the form [x, y, z].
#     """
#     rotation_matrix = quaternion_to_rotation_matrix(q)
#     return np.dot(rotation_matrix, v)

decoder = IMUinterpreter()




# calib = 0
# for t in tiempos:
#     calib += 1
#     if t > 3:
#         break


# rmeth = Rot.from_euler('XYZ', [180, 90, 0], degrees = True)

# Ra = rmeth.as_matrix()

# v1 = rmeth.as_quat()
# v1 = [v1[-1], v1[0], v1[1], v1[2]]

# rmeth = Rot.from_euler('z', 180, degrees = True)

# Rb = rmeth.as_matrix()

# v2 = rmeth.as_quat()
# v2 = [v2[-1], v2[0], v2[1], v2[2]]

# quat = [v1,v1,v2]

# # fig = plt.figure()
# # ax = plt.axes(projection='3d')

# dt =(np.average(np.diff(tiempos)))

# R0 = [[],[],[]]

# for i in range(0,3):
#     for x in [5]:
#         AXX = []
#         AYY = []
#         AZZ = []
#         UXX = []
#         UYY = []
#         UZZ = []

#         # v = np.array([0.0,0.0,0.0]).T
#         # u = np.array([0.0,0.0,0.0]).T

#         v0 = np.array([0.0,0.0,0.0]).T
#         for index in range(calib):
#             v0[0] += (data[i]['ax'][index] - v0[0])/(index+1)
#             v0[1] += (data[i]['ay'][index] - v0[1])/(index+1)
#             v0[2] += (data[i]['az'][index] - v0[2])/(index+1)
        
#         v00 = v0.copy()
#         v00 = v00/np.linalg.norm(v00)

#         rmeth = Rot.from_euler('ZY', [180, -np.rad2deg(np.arctan2(v00[0], v00[2]))], degrees = True)
#         v = rmeth.as_quat()
#         v = [v[-1], v[0], v[1], v[2]]

#         angles = MadgwickAHRS(quaternion = Quaternion(v), sampleperiod=dt, beta=x, zeta = 0)

#         for index in range(calib, len(tiempos)):
#             gyro = np.array([data[i]['gx'][index], data[i]['gy'][index], data[i]['gz'][index]])*scale
#             accel = np.array([data[i]['ax'][index], data[i]['ay'][index], data[i]['az'][index]])*scale2
#             angles.update_imu(gyro, accel)
#             q = angles.quaternion
#             # q = [q[1], q[2], q[3], q[0]]
#             q = [q[0], q[1], q[2], q[3]]
#             R0[i].append(quaternion_to_rotation_matrix(q))
#             # r, p, y = rotate_vector_by_quaternion(q, [-1,0,0])
#             axx, ayy, azz = rotate_vector_by_quaternion(q, accel)
#             # axx -= v0[0]*scale2
#             # ayy -= v0[1]*scale2
#             azz -= 9.81

#             # a = np.array([axx, ayy, azz]).T

#             # u += v*dt + 0.5 * a * (dt**2)
#             # v += a*dt

#             AXX.append(axx)
#             AYY.append(ayy)
#             AZZ.append(azz)
#             # UXX.append(u[0])
#             # UYY.append(u[1])
#             # UZZ.append(u[2])
            
#         # ax0.plot(AXX, AYY)
#         # ax0.plot(tiempos,np.rad2deg(np.arcsin(np.array(R))), label = f'IMU {i} {x}')
#         # ax1.plot(tiempos, P, label = f'IMU {i} {x}')
#         # ax2.plot(tiempos,Y, label = f'IMU {i} {x}')
#         # p = ax.scatter(UXX, UYY, UZZ, c = np.array(range(len(AXX)))/len(AXX), cmap = 'hsv')
#         # # ax.plot3D(R, P, Y)
#         # fig.colorbar(p)
#         # ax.set_xlabel('X')
#         # ax.set_ylabel('Y')
#         # ax.set_zlabel('Z')

#         # plt.figure()
#         # plt.plot(tiempos[calib:], AXX, label = 'x')
#         # plt.plot(tiempos[calib:], AYY, label = 'y')
#         # plt.plot(tiempos[calib:], AZZ, label = 'z')
#         # plt.legend()
#         # ax.scatter(0, 0, 0, c="k", s=40)

#         # ax.plot3D(1, 0, 0, c="w", alpha=0)
#         # ax.plot3D(0, 1, 0, c="w", alpha=0)
#         # ax.plot3D(0, 0, 1, c="w", alpha=0)
#         # ax.plot3D(-1, 0, 0, c="w", alpha=0)
#         # ax.plot3D(0, -1, 0, c="w", alpha=0)
#         # ax.plot3D(0, 0, -1, c="w", alpha=0)
#         # ax.set_box_aspect([1,1,1])
        

# # ax0.legend()
# # ax1.legend()
# # ax2.legend()

# # plt.show()

# tobillo = []
# rodilla = []

# debug1 = []

# for Ro0, Ro1, Ro2 in zip(R0[0], R0[1], R0[2]):
#     temp = Ra.T.dot(Ro1.T.dot(Ro2.dot(Rb.dot(np.array([1,0,0])))))
#     debug1.append(temp)
#     tobillo.append(np.rad2deg(np.arctan2(temp[2], temp[0])))
#     temp = Ra.T.dot(Ro0.T.dot(Ro1.dot(Ra.dot(np.array([1,0,0])))))
#     rodilla.append(np.rad2deg(np.arctan2(temp[2], temp[0])))


# plt.figure()
# plt.plot(tiempos[calib:], tobillo)
# plt.plot(tiempos[calib:], rodilla)
# plt.show()

# # plt.figure()
# # plt.plot(debug1)
# # plt.show()
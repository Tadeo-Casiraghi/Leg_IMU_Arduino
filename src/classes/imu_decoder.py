from classes.imu import IMU

def read_data(data_path):
    with open(data_path,'rb') as file:
        raw = file.read()
    
    packetsize = 12*3+4
    imus = []

    for _ in range(3):
        imus.append(IMU())

    decode(imus, raw, packetsize)

    with open(f'{data_path[:-4]}.csv', 'w') as file:
        file.write('Header,foot_Accel_X,foot_Accel_Y,foot_Accel_Z,foot_Gyro_X,foot_Gyro_Y,foot_Gyro_Z,shank_Accel_X,shank_Accel_Y,shank_Accel_Z,shank_Gyro_X,shank_Gyro_Y,shank_Gyro_Z,thigh_Accel_X,thigh_Accel_Y,thigh_Accel_Z,thigh_Gyro_X,thigh_Gyro_Y,thigh_Gyro_Z\n')
        for time, (fax, fay, faz), (fgx, fgy, fgz), \
                  (sax, say, saz), (sgx, sgy, sgz), \
                  (tax, tay, taz), (tgx, tgy, tgz) in \
                  zip(imus[0].times, \
                      imus[2].accel, imus[2].gyro, \
                      imus[1].accel, imus[1].gyro, \
                      imus[0].accel, imus[0].gyro):
            file.write(f'{time},{fax},{fay},{faz},{fgx},{fgy},{fgz},{sax},{say},{saz},{sgx},{sgy},{sgz},{tax},{tay},{taz},{tgx},{tgy},{tgz}\n')


def decode(imus, data, packetsize):
    pointer = 0
    while pointer < len(data)-packetsize:
        for imu in imus:
            imu.decode_data(data[pointer:(pointer + 12)])
            pointer += 12
        for imu in imus:
            imu.decode_time(data[pointer:(pointer+4)])
        pointer += 4
    
    

    

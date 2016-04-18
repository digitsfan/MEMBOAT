import smbus
import time
import math
import re
# import mathplotlib as plt

LPS_add = 0x5d  # baro and temp
LSM_add = 0x1d  # acceler and mag
L3G_add = 0x6b  # gyro

bus = smbus.SMBus(1)  # i2cdetect -y 1

# x axis pointing forward
# y axis pointing to the right
# z axis pointing down
# positive pitch, nose up
# positive roll, right wing down
# positive yaw, clockwise
sensor_sign = (1, 1, 1, -1, -1, -1, 1, 1, 1)
orientation = (1, 0, 0)


def checkSum(nmea_string):
    """ checksum for nmea string if there is no checksum present
    for simplified arduino input
    The check sum should be appended to the original NMEA sentence and
    return
    """

    # take string after $
    nmea_str = re.sub(r'^\$(.*)$', r'\1', nmea_string)
    # clear whitespace
    nmea_str = re.sub(r'\s', '', nmea_str)

    checksum = 0  # initialize
    for b in nmea_str:
        checksum ^= ord(b)  # xor

    # need to remove the front '0x' from the import hex number
    return(nmea_string + "*" +
           re.sub(r'^0x', '', hex(checksum)).zfill(2))


def sign(x):
    """ sign(x) = 1 when x >=0, sign(x) = -1 when x < 0"""
    if x >= 0:
        return 1
    else:
        return -1


def plot_xyz():
    """ not in use, for future"""
    plt.subplot(3,1,1)  # for x axis
    plt.title('x value v.s. time')
    plt.grid(True)
    plt.ylabel('X')
    plt.xlabel('t')
    plt.plot(x, '-r')

    plt.subplot(3,1,2)  # for y axis
    plt.title('y value v.s. time')
    plt.grid(True)
    plt.ylabel('Y')
    plt.xlabel('t')
    plt.plot(y, '-g')

    plt.subplot(3,1,3)  # for z axis
    plt.title('z value v.s. time')
    plt.grid(True)
    plt.ylabel('Z')
    plt.xlabel('t')
    plt.plot(z, '-b')


def twos_comp(val, bits):
    """compute the 2's compliment of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)         # compute negative value
    return val


def vector_dot(x, y):
    """ vector dot product, return a scalar
    x and y are two vectors with same length
    """

    if(len(x) != len(y)):
        raise ValueError("vector lengths differ")
    else:
        # return x1*y1+x2*y2+...xn*yn
        return sum([x[i] * y[i] for i in range(len(x))])


def vector_cross(x, y):
    """ vector cross product, return a vector
    x and y are two vectors with same length
    returned vector z is the same length as well
    this time only with dim=3
    """

    if(len(x) != len(y)):
        raise ValueError("vector lengths differ")
    elif(len(x) > 3):
        raise ValueError("vector is more than 3D")
    else:
        s = [x[1] * y[2] - x[2] * y[1],
             x[2] * y[0] - x[0] * y[2],
             x[0] * y[1] - x[1] * y[0]]
        return s


def vector_normalize(x):
    """ vector divided to its L2 norm"""
    mag = math.sqrt(vector_dot(x, x))
    return [float(i) / mag for i in x]


def LPS_single_shot(add):
    """ LPS single shot mode: from AN4450 p.16"""
    # registers
    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    TEMP_OUT_L = 0x2b
    TEMP_OUT_H = 0x2c
    PRESS_OUT_XL = 0x28
    PRESS_OUT_L = 0x29
    PRESS_OUT_H = 0x2a

    # power down the devie for clean start
    bus.write_byte_data(add, CTRL_REG1, 0x00)
    # turn on the press sensor analog frontend in single shot mode
    bus.write_byte_data(add, CTRL_REG1, 0x84)
    # wait until the meas is complete
    while True:
        # run oneshot meas
        # the setbit will be reset by the sensor itself after exe
        bus.write_byte_data(add, CTRL_REG2, 0x01)
        if bus.read_byte_data(add, CTRL_REG2) == 0x00:
            # read temp measurement
            uint_temp = (bus.read_byte_data(add, TEMP_OUT_H) << 8) + \
                bus.read_byte_data(add, TEMP_OUT_L)
            # make it signed
            int16_temp = twos_comp(int(uint_temp), 16)
            # offset and scale
            temp_degC = 42.5 + float(int16_temp) / 480

            # read press measurement
            uint_press = (bus.read_byte_data(add, PRESS_OUT_H) << 16) + \
                (bus.read_byte_data(add, PRESS_OUT_L) << 8) + \
                bus.read_byte_data(add, PRESS_OUT_XL)
            # make it signed
            int32_press = twos_comp(int(uint_press), 32)
            # offset and scale
            press_hPa = float(int32_press) / 4096
            return [temp_degC, press_hPa]
        else:
            time.sleep(0.01)


def LPS_FIFO_mean(add):
    """ LPS FIFO mode: from AN4450 p.17
     LPS25H datasheet p.28"""

    DT = 0.1
    output_timer = 0
    OUTPUT_TIMEOUT = 10
    # registers
    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    FIFO_CTRL = 0x2e
    TEMP_OUT_L = 0x2b
    TEMP_OUT_H = 0x2c
    PRESS_OUT_XL = 0x28
    PRESS_OUT_L = 0x29
    PRESS_OUT_H = 0x2a

    # power down the devie for clean start
    bus.write_byte_data(add, CTRL_REG1, 0x00)
    # turn on the press sensor analog frontend in
    # FIFO mode with ODR to be 12.5Hz (100b)
    bus.write_byte_data(add, CTRL_REG1, 0xb4)
    # run FIFO mean mode
    # with ODR =1Hz
    bus.write_byte_data(add, CTRL_REG2, 0x01)
    # mean mode 32 samples: 11001111 = 0xcf
    bus.write_byte_data(add, FIFO_CTRL, 0xcf)

    while True:
        # read temp measurement
        uint_temp = (bus.read_byte_data(add, TEMP_OUT_H) << 8) + \
            bus.read_byte_data(add, TEMP_OUT_L)
        # make it signed
        int16_temp = twos_comp(int(uint_temp), 16)
        # offset and scale
        temp_degC = 42.5 + float(int16_temp) / 480

        # read press measurement
        uint_press = (bus.read_byte_data(add, PRESS_OUT_H) << 16) + \
            (bus.read_byte_data(add, PRESS_OUT_L) << 8) + \
            bus.read_byte_data(add, PRESS_OUT_XL)
        # make it signed
        int32_press = twos_comp(int(uint_press), 32)
        # offset and scale
        press_hPa = float(int32_press) / 4096

        # sleep for a certain time
        time.sleep(DT)

        # # output result if the timer matches
        # if output_timer < OUTPUT_TIMEOUT:
        #     output_timer += 1
        # else:
        #     output_timer = 0  # reset
        #     # nmea0183 sentence
        #     temperature_pressure_sentence = "$YXXDR,C," \
        #             + str(round(temp_degC, 1)) + ",C,WCHR,P," \
        #             + str(round(press_hPa/1000.0, 3)) + ",B,STNP"
        #     yield checkSum(temperature_pressure_sentence)
        # nmea0183 sentence
        temperature_pressure_sentence = "$YXXDR,C," \
                + str(round(temp_degC, 1)) + ",C,WCHR,P," \
                + str(round(press_hPa/1000.0, 3)) + ",B,STNP"
        yield checkSum(temperature_pressure_sentence)


def LSM_acquisition(add):
    """ data acquisition of accelerator and magentometer data,
    LSM303D datasheet
    """
    # control register
    CTRL0 = 0x1f  # p.34, accelerator
    CTRL1 = 0x20
    CTRL2 = 0x21
    CTRL5 = 0x24  # p.36, magnetic
    CTRL6 = 0x25
    CTRL7 = 0x26
    FIFO_CTRL = 0x2e  # p.40
    # accelerater
    OUT_X_L_A = 0x28
    OUT_X_H_A = 0x29
    OUT_Y_L_A = 0x2a
    OUT_Y_H_A = 0x2b
    OUT_Z_L_A = 0x2c
    OUT_Z_H_A = 0x2d
    # magentic
    OUT_X_L_M = 0x08
    OUT_X_H_M = 0x09
    OUT_Y_L_M = 0x0a
    OUT_Y_H_M = 0x0b
    OUT_Z_L_M = 0x0c
    OUT_Z_H_M = 0x0d

    # follow lsm303D arduino library
    # AFS = 0, +-2g scale
    bus.write_byte_data(add, CTRL2, 0x00)
    # 50 Hz AODR, all axis enable
    bus.write_byte_data(add, CTRL1, 0x57)
    # high resolution, 6.25Hz MODR
    bus.write_byte_data(add, CTRL5, 0x64)
    # +-4 gauss scale
    bus.write_byte_data(add, CTRL6, 0x20)
    # low power mode off, continuous conversion mode
    bus.write_byte_data(add, CTRL7, 0x00)
    # # FIFO mode
    # bus.write_byte_data(add, CTRL0, 0b01000000)
    # bus.write_byte_data(add, FIFO_CTRL, 0b01000000)
    # # accelerator with 12.5Hz, all axis enable
    # bus.write_byte_data(add, CTRL1, 0b00110111)
    # # magnetic 12.5Hz, high resolutn, temp en
    # bus.write_byte_data(add, CTRL5, 0b11100000)
    # # full scale range \pm 12 gauss
    # bus.write_byte_data(add, CTRL6, 0b01101000)
    # # enable magnetic
    # bus.write_byte_data(add, CTRL7, 0x00)

    # accelerator accumulate
    while True:
        uint16_ax = (bus.read_byte_data(add, OUT_X_H_A) << 8) + \
            bus.read_byte_data(add, OUT_X_L_A)
        uint16_ay = (bus.read_byte_data(add, OUT_Y_H_A) << 8) + \
            bus.read_byte_data(add, OUT_Y_L_A)
        uint16_az = (bus.read_byte_data(add, OUT_Z_H_A) << 8) + \
            bus.read_byte_data(add, OUT_Z_L_A)

        uint16_mx = (bus.read_byte_data(add, OUT_X_H_M) << 8) + \
            bus.read_byte_data(add, OUT_X_L_M)
        uint16_my = (bus.read_byte_data(add, OUT_Y_H_M) << 8) + \
            bus.read_byte_data(add, OUT_Y_L_M)
        uint16_mz = (bus.read_byte_data(add, OUT_Z_H_M) << 8) + \
            bus.read_byte_data(add, OUT_Z_L_M)
        # accelerometer 12 bit left aligned
        # ax = twos_comp(uint16_ax>>4, 12)
        # ay = twos_comp(uint16_ay>>4, 12)
        # az = twos_comp(uint16_az>>4, 12)
        ax = twos_comp(uint16_ax, 16)
        ay = twos_comp(uint16_ay, 16)
        az = twos_comp(uint16_az, 16)

        mx = twos_comp(uint16_mx, 16)
        my = twos_comp(uint16_my, 16)
        mz = twos_comp(uint16_mz, 16)

        yield [ax, ay, az, mx, my, mz]


def LSM_offset(add, timer_out=1000):
    """ read the LSM value for certain time and return
    the value and write to offset register,
    make sure the chip is leveled and calm
    lsm arduino library
    """

    # initialize
    timer = 0
    m_min = [32767, 32767, 32767]
    m_max = [-32768, -32768, -32768]
    # lsm reading
    lsm_result = LSM_acquisition(add)
    # loop to update min and max
    while timer <= timer_out:
        m = lsm_result.next()[3:6]
        m_min = [min(m_min[i], m[i]) for i in range(3)]
        m_max = [max(m_max[i], m[i]) for i in range(3)]
        timer += 1
        print [m_min, m_max]

    return [m_min, m_max]


def L3G_acquisition(add):
    """read data from L3GD20H gyro,
    AN4506"""

    # control register
    CTRL_REG1 = 0x20
    CTRL_REG4 = 0x23
    LOW_ODR = 0x39
    FIFO_CTRL = 0x2e
    # output register
    OUT_X_L = 0x28
    OUT_X_H = 0x29
    OUT_Y_L = 0x2a
    OUT_Y_H = 0x2b
    OUT_Z_L = 0x2c
    OUT_Z_H = 0x2d

    # low odr mode, 50Hz, 2000 dps full scale
    bus.write_byte_data(add, CTRL_REG1, 0b10001111)
    bus.write_byte_data(add, CTRL_REG4, 0b00110000)
    bus.write_byte_data(add, LOW_ODR, 0b00000001)
    bus.write_byte_data(add, FIFO_CTRL, 0b01000000)

    # accelerator accumulate
    while True:
        uint16_gx = (bus.read_byte_data(add, OUT_X_H) << 8) + \
            bus.read_byte_data(add, OUT_X_L)
        uint16_gy = (bus.read_byte_data(add, OUT_Y_H) << 8) + \
            bus.read_byte_data(add, OUT_Y_L)
        uint16_gz = (bus.read_byte_data(add, OUT_Z_H) << 8) + \
            bus.read_byte_data(add, OUT_Z_L)

        gx = twos_comp(uint16_gx, 16)
        gy = twos_comp(uint16_gy, 16)
        gz = twos_comp(uint16_gz, 16)

        yield [gx, gy, gz]


def heading(add):
    """ use arduino's LSM303 library,
    orientation is defined as a 3-d vector,
    (+-1, +-1, +-1) for x, y, z axis,
    default is (+1, 0, 0) for LSM303D
    """

    # calculate max and min mag readings
    output_timer = 0
    OUTPUT_TIMEOUT = 10
    DT = 0.1
    calibration_timer_out = 100
    mag_min, mag_max = LSM_offset(add, calibration_timer_out)
    # mag_min = (-2838, -3539, -4191)
    # mag_max = (2133, 970, 534)

    # calculate average offset
    mag_ave = [(mag_min[i] + mag_max[i]) / 2.0 for i in range(len(mag_min))]

    # calculate E and N
    # first obtain acc and mag raw data
    lsm_result = LSM_acquisition(add)

    while True:
        tmp = lsm_result.next()
        acc = tmp[0:3]
        mag = tmp[3:6]
        temp_mag = [(mag[i] - mag_ave[i]) for i in range(3)]
        E = vector_cross(temp_mag, acc)
        E_norm = vector_normalize(E)

        N = vector_cross(acc, E_norm)
        N_norm = vector_normalize(N)

        heading = math.degrees(math.atan2(vector_dot(E_norm, orientation),
                                          vector_dot(N_norm, orientation)))

        # correct for negative degree
        if heading < 0:
            heading += 360
        # sleep for a certain time
        time.sleep(DT)

        # # output result if the timer matches
        # if output_timer < OUTPUT_TIMEOUT:
        #     output_timer += 1
        # else:
        #     output_timer = 0  # reset
        #     # nmea0183 sentence
        #     heading_sentence = "$HCTHS," + str(round(heading, 2)) + ",A"
        #     # output
        #     yield checkSum(heading_sentence)

        # nmea0183 sentence
        heading_sentence = "$HCTHS," + str(round(heading, 2)) + ",A"
        yield checkSum(heading_sentence)


def pitch_roll_complementary(LSM_add, L3G_add, sensor_sign, isnmea=True):
    """ based on lsm and l3g's data,
    calculate useable angles and further
    ozzmarker.com/2013/04/29/guide-to-interfacing-a-gyro-and-accelerometer-with
    -a-raspberry-pi/
    """
    A_GAIN = 0.061 / 1000 # +-2g scale, mg/LSB
    G_GAIN = 70.0 / 1000  # 2000 dps, 70 mdps/digit, p.9
    DT = 0.02  # 12.5 Hz = 0.08s, 50Hz = 0.02 loop period
    wGyro = 5.0  # complementary filer coef
    acc_offset = [0, 0, 0]
    Rest_new = [0, 0, 0]
    R_gyro = [0, 0, 0]
    offset_timer = 0
    output_timer = 0
    OFFSET_TIMEOUT = 100
    OUTPUT_TIMEOUT = 50
    initial_flag = 0

    lsm_result = LSM_acquisition(LSM_add)
    l3g_result = L3G_acquisition(L3G_add)

    while True:
        # raw data
        acc = lsm_result.next()[0:3]
        gyro = l3g_result.next()
        # data in their unit, acc: g gyro: degree/s
        acc_g = [acc[i] * A_GAIN * sensor_sign[i+3] for i in range(3)]
        gyro_dps = [gyro[i] * G_GAIN * sensor_sign[i+6] for i in range(3)]

        # offset
        if offset_timer < OFFSET_TIMEOUT:
            # print gyro_dps
            acc_offset = [acc_offset[i] + acc_g[i] for i in range(3)]
            # gyro_offset = [gyro_offset[i] + gyro_dps[i] for i in range(3)]
            offset_timer += 1
            continue
        else: # offset done
            # take average
            acc_offset_ave = [acc_offset[i] / OFFSET_TIMEOUT \
                    for i in range(3)]
            # minus from the reading
            acc_ave = [acc_g[i] - acc_offset_ave[i] for i in range(3)]
            # z axis must be offset to 1g
            acc_ave[2] += 1
            # normalize acc_ave
            acc_ave = vector_normalize(acc_ave)

            # update R_est
            if initial_flag == 0:
                Rest_old = acc_ave
                initial_flag = 1
            else:
                Rest_old = Rest_new

        # accelerator angle in radian
        Axz_old = math.atan2(Rest_old[0], Rest_old[2])
        Ayz_old = math.atan2(Rest_old[1], Rest_old[2])

        # update angle with gyro readings
        Axz = Axz_old + math.radians(gyro_dps[0] * DT)
        Ayz = Ayz_old + math.radians(gyro_dps[1] * DT)
        # pitch and roll
        pitch = math.degrees(Axz)
        roll = math.degrees(Ayz)

        # calculate gyro angle
        R_gyro[0] = math.sin(Axz) / math.sqrt(1 + math.cos(Axz) ** 2 \
               * math.tan(Ayz) ** 2)
        R_gyro[1] = math.sin(Ayz) / math.sqrt(1 + math.cos(Ayz) ** 2 \
               * math.tan(Axz) ** 2)
        R_gyro[2] = sign(Rest_old[2]) * \
                math.sqrt(1 - R_gyro[0] ** 2 - R_gyro[1] ** 2)

        # complementary filter
        Rest_new = [(acc_ave[i] + R_gyro[i] * wGyro) / (1+wGyro) \
                for i in range(3)]
        # normalize
        Rest_new = vector_normalize(Rest_new)
        # sleep for DT time
        time.sleep(DT)

        # # output result if the timer matches
        # if output_timer < OUTPUT_TIMEOUT:
        #     output_timer += 1
        # else:
        #     output_timer = 0  # reset
        #     # output
        #     # print "ACC: ", [round(i, 2) for i in acc_ave]
        #     # print "Gyr: ", [round(i, 2) for i in R_gyro]
        #     # print "AN: ", [round(i, 2) for i in Rest_new]
        #     # print "Pitch: ", round(pitch, 0), "Roll: ", round(roll, 0)
        #     # nmea0183 format text
        #     pitch_roll_sentence = "$YXXDR,A," \
        #             + str(round(pitch, 1)) + ",D,PTCH,A," \
        #             + str(round(roll, 1)) + ",D,ROLL"
        #     acc_sentence = "$YXXDR,A," \
        #             + str(round(acc_g[0],3)) + ",G,XACC,A," \
        #             + str(round(acc_g[1],3)) + ",G,YACC,A," \
        #             + str(round(acc_g[2],3)) + ",G,ZACC"
        #     yield checkSum(pitch_roll_sentence) + "\r\n" + \
        #             checkSum(acc_sentence)
        if isnmea:
            # nmea0183 format text
            pitch_roll_sentence = "$YXXDR,A," \
                    + str(round(pitch, 1)) + ",D,PTCH,A," \
                    + str(round(roll, 1)) + ",D,ROLL"
            acc_sentence = "$YXXDR,A," \
                    + str(round(acc_g[0],3)) + ",G,XACC,A," \
                    + str(round(acc_g[1],3)) + ",G,YACC,A," \
                    + str(round(acc_g[2],3)) + ",G,ZACC"
            yield checkSum(pitch_roll_sentence) + "\r\n" + \
                    checkSum(acc_sentence)
        else:
            yield ({"pitch": pitch,
                    "roll": roll})



def print_raw_lsm_l3g(LSM_add, L3G_add, sensor_sign):
    G_GAIN = 0.07  # dps/LSB
    A_GAIN = 0.061 / 1000  # mg/LSB
    M_GAIN = 0.16 / 1000  # mgauss/LSB
    lsm_result = LSM_acquisition(LSM_add)
    l3g_result = L3G_acquisition(L3G_add)
    while True:
        acc = lsm_result.next()[0:3]
        mag = lsm_result.next()[3:6]
        gyro = l3g_result.next()
        mag_gauss = [round(mag[i] * M_GAIN * sensor_sign[i], 2) \
                for i in range(3)]
        acc_g = [round(acc[i] * A_GAIN * sensor_sign[i+3], 2) \
                for i in range(3)]
        gyro_dps = [round(gyro[i] * G_GAIN * sensor_sign[i+6], 2) \
                for i in range(3)]

        # pitch = math.asin(-1.0 * acc_g[0])
        # roll = math.asin(acc_g[1] / math.cos(pitch))
        # if mag_gauss[0] > 0 and mag_gauss[1] > 0:
        #     heading = math.atan(mag_gauss[1] / mag_gauss[0])
        # elif mag_gauss[0] < 0:
        #     heading = 180 + math.atan(mag_gauss[1] / mag_gauss[0])
        # elif mag_gauss[0] > 0 and mag_gauss[1] <= 0:
        #     heading = 360 + math.atan(mag_gauss[1] / mag_gauss[0])
        # elif mag_gauss[0] == 0 and mag_gauss[1] < 0:
        #     heading = 90
        # elif mag_gauss[0] == 0 and mag_gauss[1] >= 0:
        #     heading = 270
        # print pitch, roll, heading

        print acc_g

        time.sleep(0.5)


if __name__ == '__main__':
    a = pitch_roll_complementary(LSM_add, L3G_add, sensor_sign,isnmea=False)
    while True:
        print a.next()
        # time.sleep(1)
    # heading(LSM_add)
    # print_raw_lsm_l3g(LSM_add, L3G_add, sensor_sign)
    # for lps_result in LPS_single_shot(LPS_add):
    #     print lps_result
    #     time.sleep(0.1)
    # for lsm_result in LSM_acquisition(LSM_add):
    #     print lsm_result
    #     time.sleep(0.1)
    # for lsm_result in LSM_acquisition(LSM_add):
    #     print lsm_result
    #     time.sleep(0.1)

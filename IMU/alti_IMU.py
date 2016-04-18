import smbus
import time
import math

LPS_add = 0x5d  # baro and temp
LSM_add = 0x1d  # acceler and mag
L3G_add = 0x6b  # gyro

bus = smbus.SMBus(1)  # i2cdetect -y 1


def twos_comp(val, bits):
    """compute the 2's compliment of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)         # compute negative value
    return val


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
            yield [temp_degC, press_hPa]
        else:
            time.sleep(0.01)


def LPS_FIFO_mean(add):
    """ LPS FIFO mode: from AN4450 p.17
     LPS25H datasheet p.28"""
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
        yield [temp_degC, press_hPa]


def LSM_acquisition(add):
    """ data acquisition of accelerator and magentometer data,
    LSM303D datasheet
    """
    # control register
    CTRL0 = 0x1f  # p.34, accelerator
    CTRL1 = 0x20
    CTRL5 = 0x24  # p.36, magnetic
    CTRL6 = 0x25
    CTRL7 = 0x26
    FIFO_CTRL = 0x2e  #p.40
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

    # FIFO mode
    bus.write_byte_data(add, CTRL0, 0b01000000)
    bus.write_byte_data(add, FIFO_CTRL, 0b01000000)
    # accelerator with 12.5Hz, all axis enable
    bus.write_byte_data(add, CTRL1, 0b00110111)
    # magnetic 12.5Hz, high resolutn, temp en
    bus.write_byte_data(add, CTRL5, 0b11100000)
    # full scale range \pm 12 gauss
    bus.write_byte_data(add, CTRL6, 0b01101000)
    # enable magnetic
    bus.write_byte_data(add, CTRL7, 0x00)

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

        ax = twos_comp(uint16_ax, 16)
        ay = twos_comp(uint16_ay, 16)
        az = twos_comp(uint16_az, 16)

        mx = twos_comp(uint16_mx, 16)
        my = twos_comp(uint16_my, 16)
        mz = twos_comp(uint16_mz, 16)

        yield [ax, ay, az, mx, my, mz]


def LSM_offset(add, timer_out = 1000):
    """ read the LSM value for certain time and return
    the value and write to offset register,
    make sure the chip is leveled and calm
    """
    # initialize
    timer = 0
    a_min = [32767, 32767, 32767]
    a_max = [-32768, -32768, -32768]
    # loop to update min and max
    for a in LSM_acquisition(add)[0:3]:
        a_min = [min(a_min[i], a[i]) \
                for i in range(0,3)]
        a_max = [max(a_max[i], a[i]) \
                for i in range(0,3)]
        timer += 1
        # check if timer reached
        if timer >= timer_out:
            return [a_min, a_max]

        
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
    
    # low odr mode, 12.5Hz, 2000 dps full scale
    bus.write_byte_data(add, CTRL_REG1, 0b00001111)
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


def calculation(LSM_add, L3G_add):
    """ based on lsm and l3g's data,
    calculate useable angles and further
    """
    G_GAIN = 0.07  # 2000 dps, 70 mdps/digit, p.9
    DT = 0.08  # 12.5 Hz = 0.08s, loop period
    AA = 0.98  # complementary filer coef
    x_angle, y_angle, z_angle = 0, 0, 0

    lsm_result = LSM_acquisition(LSM_add)
    l3g_result = L3G_acquisition(L3G_add)

    while True:
        # print lsm_result.next()
        # print l3g_result.next()
        # gyro angle in degree
        rate_gyro = [i * G_GAIN for i in l3g_result.next()]
        gyro_angle = [i * DT for i in rate_gyro]
        
        # accelerator angle in degree
        acc = lsm_result.next()[0:3]
        accx_angle = math.degrees(math.atan2(acc[1], acc[2]) + math.pi)
        accy_angle = math.degrees(math.atan2(acc[2], acc[0]) + math.pi)
        accz_angle = math.degrees(math.atan2(acc[0], acc[1]) + math.pi)
        acc_angle = [accx_angle, accy_angle, accz_angle]

        # complementary filter:
        # current angle = 98% (current angle + gyro rate) + 
        #                 2% acc angle
        
        x_angle = AA * (x_angle + gyro_angle[0]) + \
                (1 - AA) * acc_angle[0]
        y_angle = AA * (y_angle + gyro_angle[1]) + \
                (1 - AA) * acc_angle[1]
        z_angle = AA * (z_angle + gyro_angle[2]) + \
                (1 - AA) * acc_angle[2]
        
        print [x_angle, y_angle, z_angle]


if __name__ == '__main__':
    calculation(LSM_add, L3G_add)
    # lps_result = LPS_FIFO_mean(LPS_add)
    # lsm_result = LSM_acquisition(LSM_add)
    # l3g_result = L3G_acquisition(L3G_add)
    # while True:
    #     print lps_result.next()
    #     print lsm_result.next()
    #     print l3g_result.next()
    #     time.sleep(1)

    # for lps_result in LPS_single_shot(LPS_add):
    #     print lps_result
    #     time.sleep(0.1)
    # for lsm_result in LSM_acquisition(LSM_add):
    #     print lsm_result
    #     time.sleep(0.1)
    # for lsm_result in LSM_acquisition(LSM_add):
    #     print lsm_result
    #     time.sleep(0.1)

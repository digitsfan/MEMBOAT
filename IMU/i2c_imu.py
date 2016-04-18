import smbus
import time

LPS_add = 0x5d  # baro and temp
LSM_add = 0x1d  # acceler and mag
L3G_add = 0x6b  # gyro

bus = smbus.SMBus(1)  # i2cdetect -y 1

# global registers
CTRL_REG1 = 0x20
CTRL_REG2 = 0x21
TEMP_OUT_L = 0x2b
TEMP_OUT_H = 0x2c
PRESS_OUT_XL = 0x28
PRESS_OUT_L = 0x29
PRESS_OUT_H = 0x2a


def twos_comp(val, bits):
    """compute the 2's compliment of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)         # compute negative value
    return val


def LPS_single_shot(add):
    # power down the devie for clean start
    bus.write_byte_data(add, CTRL_REG1, 0x00)
    # turn on the press sensor analog frontend in single shot mode
    bus.write_byte_data(add, CTRL_REG1, 0x84)
    # run oneshot meas the setbit will be reset by the sensor itself after exe
    bus.write_byte_data(add, CTRL_REG2, 0x01)
    # wait untile the meas is complete
    one_shot_mode = True
    while one_shot_mode:
        if bus.read_byte_data(add, CTRL_REG2) == 0x00:
            # read temp measurement
            uint_temp = bus.read_byte_data(add, TEMP_OUT_H) << 8 + \
                bus.read_byte_data(add, TEMP_OUT_L)
            # make it signed
            int16_temp = twos_comp(int(uint_temp), 16)
            # offset and scale
            temp_degC = 42.5 + float(int16_temp) / 480

            # read press measurement
            uint_press = bus.read_byte_data(add, PRESS_OUT_H) << 16 + \
                bus.read_byte_data(add, PRESS_OUT_L) << 8 + \
                bus.read_byte_data(add, PRESS_OUT_XL)
            # make it signed
            int32_press = twos_comp(int(uint_press), 32)
            # offset and scale
            press_hPa = float(int32_press) / 4096
            one_shot_mode = False
	    return {'temperature': temp_degC, 'pressure': press_hPa}
        else:
            time.sleep(0.5)


if __name__ == '__main__':
    result = LPS_single_shot(LPS_add)
    print "temperature = {temperature}C, \
    pressure = {pressure}hPa".format(**result)

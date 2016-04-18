#! /usr/bin/python
# 2015/11/16
# top level to log GPS and IMU data
# into NMEA sentences
# need to use I2C/alti_IMU.py, GPS/SC16IS750.py, NMEA/file_IO.py

import I2C.alti_IMU as IMU
import GPS.SC16IS750 as GPS
import NMEA.file_IO as FLE

""" IMU: LPS_FIFO_mean(LPS_add),
         heading(LSM_add)
         pitch_roll_complementary(LSM_add, L3G_add, sensor_sign)
         checkSum(nmea_String)
    GPS: read_sentence_SC16(SC16_add)
         GPS_I2C(add, update_rate, output_data)
    FLE: modify_filename(filename, extension, is_append_time=True)
"""

def log_data(filename, *args):
    """ log data into a file
    that all sentence generators in the args can be
    logged in sequence"""

    with open(filename, 'w') as _lf:
        try:
            while (True):
                for gen in args:
                    msg = gen.next()
                    print(msg)  # debug
                    _lf.write(msg + '\n')  # write sentence to log file
        except KeyboardInterrupt:
            _lf.close()  # close file

# assign generator to global variables
# magnetic heading IMU
heading_gen = IMU.heading(IMU.LSM_add)
# pitch and roll from IMU
pitch_roll_gen = IMU.pitch_roll_complementary(IMU.LSM_add, \
        IMU.L3G_add, IMU.sensor_sign)
# temperature and pressure from IMU
temp_press_gen = IMU.LPS_FIFO_mean(IMU.LPS_add)
# GPS from GPS
gps_gen = GPS.GPS_I2C(GPS.SC16_add, 1, "RMCONLY")

# define filename
filename = 'GPS_IMU.log'

while True:
    log_data(FLE.modify_filename(filename, 'log'), \
            gps_gen, heading_gen, pitch_roll_gen, temp_press_gen)


#! usr/bin/python
# python to connect, update and retrive database
# 2015/10/06
# Ren Ye

import mysql.connector

""" connect to mysql server,
update tables from the nmea sentence result
retrieve data
"""


def connect_mysql(config_filename):
    """ This is to connect the mysql database so that the parsed sentence can
    be written to the specific tables.
    to read from a configration data file
    """

    # read configurations from a file
    conn_params = {}  # initialize a dict
    with open(config_filename, 'r') as config_file:
        for line in config_file:
            key, value = line.split(None, 2)
            conn_params[key] = value

    try:
        conn = mysql.connector.connect(**conn_params)
        print("Connected")
        return conn
    except mysql.connector.Error as e:
        print("Cannot connect to server")
        print("Error code: %s" % e.errno)
        print("Error message: %s" % e.msg)
        print("Error SQLSTATE: %s" % e.sqlstate)
        return None


def write_mysql():
    """ with the opened database, when passing each sentence, the information
    can be written into the tables identified by the talker and type

    The structure of the database should be
    Table GGA:
        id
        timestamp(timestamp)
        latitude(float, signed)
        longitude(float, signed)
        quality(int)
        satellite(int, number)
        altitude(float, meter)
        geo_sep(float, meter)

    Table VTG:
        id
        true_track(int)
        mag_track(int)
        speed(float, km/h)
        FAA(char)
    Table MWV:
        id
        wind_angle(float)
        wind_speed(float, m/s)
    Table MDA:
        id
        pressure(float,bar)
        air_temperature(float,C)
        water_temperature(float,C)
        relative_humidity(float,%)
        absolute_humidity(float)
        dew_point(float,C)
        magnetic_wind_angle(float)
        true_wind_angle(float) as TWAG
        wind_speed(float, m/s) as WSPD
    Table ZDA:
        id
        timestamp(timestamp)
    Table XDR:
        id
        pitch(float,degree) as PTCH
        roll(float,degree) as ROLL
        wind_chill(float, degreeC) as WCHR
        theoretical_wind_chill(float, degreeC) as WCHT
        heat_index(float, degreeC) as HINX
        station_pressure(float, bar) as STNP

    $YXXDR,C,,C,WCHR,C,,C,WCHT,C,,C,HINX,P,0.9954,B,STNP*4A
    """



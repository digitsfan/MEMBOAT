USE DATABASE NMEA; # use the NMEA database
DROP TABLE IF EXISTS GPS;
CREATE TABLE GPS
(
id INT UNSIGNED NOT NULL UNIQUE AUTO_INCREMENT, # unique id, primary key
# from $GPGGA (p. 11)
tstamp TIMESTAMP UNIQUE, #utc
latitude FLOAT,
longitude FLOAT,
altitude FLOAT,
quality INT UNSIGNED, # [0,8]
satellite INT UNSIGNED, # [0-12]

# from $GPVTG (p. 26)
t_cog FLOAT, # true course over ground
m_cog FLOAT, # magnetic course over ground
e_sog FLOAT, # speed over ground, empirical knots
m_sog FLOAT, # speed over ground, metric, km/h
PRIMARY KEY (id)

# notice these data can be made from $GPRMC and it is preferred (p. 23)
# notice timestamp data can be made from $GPZDA (p. 40)
);


DROP TABLE IF EXISTS METEOROLOGY;
CREATE TABLE METEORLOGY
(
id INT UNSIGNED NOT NULL UNIQUE AUTO_INCREMENT, # unique id, primary key
# from $WIMDA (p. 19)
e_pressure FLOAT, # empirical, inch of mecury
m_pressure FLOAT, # metric, bar
air_temperature FLOAT, # degree C
water_temperature FLOAT, # degree C
r_humidity FLOAT, # relative, percentage
dewpoint FLOAT, # degree C
t_wind_direction FLOAT, # true
m_wind_direction FLOAT, # magnetic
e_wind_speed FLOAT, # empirical, knots
s_wind_speed FLOAT, # metric, m/s

# from $WIMWV (p. 22), relative to vessel's centerline
t_wind_direction_vessel FLOAT, # true, vessel's centerline
m_wind_direction_vessel FLOAT, # magnetic, vessel's centerline
t_e_wind_speed FLOAT, # true wind speed empirical, knots
a_e_wind_speed FLOAT, # apparent wind speed empirical, knots

# from $WIMWD (p. 21), relative to true north
t_wind_direction_north FLOAT, # true, north
m_wind_direction_north FLOAT, # magnetic, north

# these two filed can also give wind speed reading but will conflict
PRIMARY KEY (id)
);

DROP TABLE IF EXISTS TRANSDUCER;
CREATE TABLE TRANSDUCER
(
id INT UNSIGNED NOT NULL UNIQUE AUTO_INCREMENT, # unique id, primary key
# from $YXXDR-A (p. 30)
t_wind_chill FLOAT, # theoretical, degree C
r_wind_chill FLOAT, # relative, degree C
heat_index FLOAT, # heat index, degree C
station_pressure FLOAT, # bar

# from $YXXDR-B (p. 32)
roll FLOAT, # roll, degree
pitch FLOAT, # pitch, degree

# from $HCTHS (p. 25)
heading FLOAT, # true heading to true north

PRIMARY KEY (id)
);


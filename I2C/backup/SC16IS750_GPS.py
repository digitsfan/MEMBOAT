class GPS_I2C(SC16IS750):
    """ Adafruit Ultimate GPS is a uart device,
    use SC16IS750 to log it via I2C,
    the received data should be ASCII or UTF-8
    can be either yield or return or write to log file """

    # NMEA GPS setup sentences
    # update rate
    PMTK_SET_NMEA_UPDATE_1HZ = '$PMTK220,1000*1F\r\n'
    PMTK_SET_NMEA_UPDATE_5HZ = '$PMTK220,200*2C\r\n'
    PMTK_SET_NMEA_UPDATE_10HZ = '$PMTK220,100*2F\r\n'
    # output data
    # RMC only
    PMTK_SET_NMEA_OUTPUT_RMCONLY = \
        '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n'
    PMTK_SET_NMEA_OUTPUT_RMCGGA =  \
        '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n'
    PMTK_SET_NMEA_OUTPUT_ALLDATA = \
        '$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n'
    PMTK_SET_NMEA_OUTPUT_OFF = \
        '$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n'

    # get version
    PMTK_Q_RELEASE = '$PMTK605*31\r\n'

    def __init__(self, address, update_rate=1, output_data="RMCONLY"):
        self.add = address
        self.update_rate = update_rate
        self.output_data = output_data

    def GPS_initialize(self):
        """ initialize GPS """
        # include i2c chip initialization
        self.initialize()

        # write update rate to RX of UART
        if self.update_rate == 1:
            self.write_sentence(self.PMTK_SET_NMEA_UPDATE_1HZ)
        elif self.update_rate == 5:
            self.write_sentence(self.PMTK_SET_NMEA_UPDATE_5HZ)
        elif self.update_rate == 10:
            self.write_sentence(self.PMTK_SET_NMEA_UPDATE_10HZ)
        else:
            raise ValueError("Update rate not supported")

        # write sentence output to RX of UART
        if self.output_data == 'RMCONLY':
            self.write_sentence(self.PMTK_SET_NMEA_OUTPUT_RMCONLY)
        elif self.output_data == 'RMCGGA':
            self.write_sentence(self.PMTK_SET_NMEA_OUTPUT_RMCGGA)
        elif self.output_data == 'ALLDATA':
            self.write_sentence(self.PMTK_SET_NMEA_OUTPUT_ALLDATA)
        else:
            raise ValueError("Output data not supported")

        # write q release to RX of UART
        self.write_sentence(self.PMTK_Q_RELEASE)
        # check the q realease
        GPS_sentence = self.read_sentence()
        print GPS_sentence
        # sleep for a short while
        time.sleep(1.0/self.update_rate)

    def GPS_read_sentence(self):
        while True:
            GPS_sentence = self.read_sentence()
            print GPS_sentence
            # sleep for a short while
            time.sleep(1.0/self.update_rate)


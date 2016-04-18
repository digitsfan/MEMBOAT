#! /usr/bin/python
# 2015/10/19
# use I2C to uart bridge to log GPS data
# and future for nmea device data
# 2015/12/23
# change to class type

import smbus
import time
import re

""" I2C to UART bridge with SC16IS750
I2C bus address is 0x4d
it can also be used as SPI to UART bridge
"""

# enable I2C bus
bus = smbus.SMBus(1)

# define a class
class SC16IS750(object):
    """ class of SC16IS750 """

    # registers
    XHR = 0x00  # read/write holding
    FCR = 0x02  # FIFO control
    LCR = 0x03  # line control
    MCR = 0x04  # modem control
    MSR = 0x05  # modem status
    LSR = 0x06  # line status
    TXLVL = 0x08  # transmit FIFO level
    RXLVL = 0x09  # receive FIFO level
    DLL = 0x00  # divider latch, only when LCR[7] = 0
    DLH = 0x01  # divider latch, only when LCR[7] = 0
    EFR = 0x02  # enhanced feature register, only when LCE[7] = 0
    TLR = 0x07  # trigger level register, only when EFR[4] = 1 & MCR[2] = 1

    def __init__(self, address):
        """ pass initial parameter to the class object """
        self.add = address

    def I2C_initialize(self, baudrate=9600):
        """ initialize chip,
        TODO: add parameters to be changed """

        # configure UART register
        # LCR, enable DLL and DLH write
        self.write_byte(self.LCR, 0b10000011)
        # assume 9600 baud
        # 16*9600 = 14.7456MHz/96
        self.write_byte(self.DLL, int(14.7456e6/(16*baudrate)))
        self.write_byte(self.DLH, 0)
        # enhanced feature
        self.write_byte(self.EFR, 0b00010000)
        # LCR, disable DLL and DLH write, 8b wl, 1b sb, no p
        self.write_byte(self.LCR, 0b00000011)

        # MCR, normal
        self.write_byte(self.MCR, 0b00000100)
        # reset FIFO
        self.write_byte(self.FCR, 0x06)
        # enable FIFO
        self.write_byte(self.FCR, 0b00000111)
        # TLR to be 4, fastest refreshing
        self.write_byte(self.TLR, 0b00010001)

        # check status, debug
        print "FCR: {0:x}".format(self.read_byte(self.FCR))
        print "LCR: {0:x}".format(self.read_byte(self.LCR))
        print "MCR: {0:x}".format(self.read_byte(self.MCR))
        print "LSR: {0:x}".format(self.read_byte(self.LSR))
        print "MSR: {0:x}".format(self.read_byte(self.MSR))

    def write_byte(self, reg, value):
        """SC16IS7X0 expects a R/W first, followd by a
        4 bit register address and combine with a value
        """
        Write_bit = 0b00000000
        # left shift by 3 bit
        reg = reg << 3
        # bitwise or with a write bit
        actual_reg = reg | Write_bit
        bus.write_byte_data(self.add, actual_reg, value)

    def read_byte(self, reg):
        """SC16IS7X0 expects a R/W first, followd by a
        4 bit register address and combine with a value
        """
        Read_bit = 0b10000000
        # left shift by 3 bit
        reg = reg << 3
        # bitwise or with a write bit
        actual_reg = reg | Read_bit
        return bus.read_byte_data(self.add, actual_reg)

    def write_sentence(self, sentence):
        """write a sentence to the UART,
        must end with \r\n"""
        for i in range(0, len(sentence)):
            self.write_byte(self.XHR, ord(sentence[i]))

    def read_sentence(self):
        """ read sentence from the FIFO,
        sentence ends with \r\n
        sentence output to be ascii """
        sentence = ''  # initialize

        # until newline at end of sentence
        while not re.search(r'\n$', sentence):
            # read FIFO length
            work = self.read_byte(self.RXLVL)
            # debug
            # if work > 0:
            #     print "FIFO length: {0}".format(work)
            # loop to push fifo data into sentence
            while 0 < work:
                sentence += chr(self.read_byte(self.XHR))
                work -= 1

        # after sentence complete, print out
        if len(sentence) > 0:
            return sentence
            # yield sentence
        else:
            return None
            # yield None


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


class RS485_I2C(SC16IS750):
    """ RS485 to TTL connected to SC16
    make sure that the baudrate is set to 9600
    """

    def RS485_read_sentence(self, update_rate = 0.01):
        while True:
            sentence = self.read_sentence()
            print sentence
            # sleep for a short while
            time.sleep(update_rate)

if __name__ == "__main__":
    GPS = GPS_I2C(0x48)
    GPS.I2C_initialize()
    GPS.GPS_initialize()
    # GPS.GPS_read_sentence()
    RS485 = RS485_I2C(0x49)
    RS485.I2C_initialize()
    RS485.RS485_read_sentence()



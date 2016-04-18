import smbus
import time
import re

""" I2C to UART bridge with SC16IS750
I2C bus address is 0x4d
"""

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

# enable I2C bus
bus = smbus.SMBus(1)
add = 0x4d


def write_byte_SC16(add, reg, value):
    """SC16IS7X0 expects a R/W first, followd by a
    4 bit register address and combine with a value
    """
    Write_bit = 0b00000000
    # left shift by 3 bit
    reg = reg << 3
    # bitwise or with a write bit
    actual_reg = reg | Write_bit
    bus.write_byte_data(add, actual_reg, value)


def read_byte_SC16(add, reg):
    """SC16IS7X0 expects a R/W first, followd by a
    4 bit register address and combine with a value
    """
    Read_bit = 0b10000000
    # left shift by 3 bit
    reg = reg << 3
    # bitwise or with a write bit
    actual_reg = reg | Read_bit
    return bus.read_byte_data(add, actual_reg)


def write_sentence_SC16(add, reg, sentence):
    """write a sentence to the UART,
    must end with \r\n"""
    for i in range(0, len(sentence)):
        write_byte_SC16(add, XHR, ord(sentence[i]))


def read_sentence_SC16(add, level_reg=RXLVL, buffer_reg=XHR):
    """ read sentence from the FIFO,
    sentence ends with \r\n
    sentence output to be ascii
    """

    sentence = ''  # initialize

    # until newline at end of sentence
    while not re.search(r'\n$', sentence):
        # read FIFO length
        work = read_byte_SC16(add, level_reg)
        # debug
        # if work > 0:
        #     print "FIFO length: {0}".format(work)
        # loop to push fifo data into sentence
        while 0 < work:
            sentence += chr(read_byte_SC16(add, buffer_reg))
            work -= 1

    # after sentence complete, print out
    if len(sentence) > 0:
        return sentence
    else:
        return None


def write_log(filename, add):
    """ write to log file from I2C address"""
    with open('log.log', 'w') as _wf:
        try:
            while True:
                sentence = read_sentence_SC16(add)
                _wf.write(sentence)
        except KeyboardInterrupt:
            _wf.close()


# configure UART register
# LCR, enable DLL and DLH write
write_byte_SC16(add, LCR, 0b10000011)
# assume 9600 baud
# 16*9600 = 14.7456MHz/96
write_byte_SC16(add, DLL, 96)
write_byte_SC16(add, DLH, 0)
# enhanced feature
write_byte_SC16(add, EFR, 0b00010000)
# LCR, disable DLL and DLH write, 8b wl, 2b sb, no p
write_byte_SC16(add, LCR, 0b00000011)

# MCR, normal
write_byte_SC16(add, MCR, 0b00000100)
# reset FIFO
write_byte_SC16(add, FCR, 0x06)
# enable FIFO
write_byte_SC16(add, FCR, 0b00000111)
# TLR to be 4, fastest refreshing
write_byte_SC16(add, TLR, 0b00010001)

# check status, debug
# print "FCR: {0:x}".format(read_byte_SC16(add, FCR))
# print "LCR: {0:x}".format(read_byte_SC16(add, LCR))
# print "MCR: {0:x}".format(read_byte_SC16(add, MCR))
# print "LSR: {0:x}".format(read_byte_SC16(add, LSR))
# print "MSR: {0:x}".format(read_byte_SC16(add, MSR))

# write update rate to RX of UART
write_sentence_SC16(add, XHR, PMTK_SET_NMEA_UPDATE_1HZ)

# write sentence output to RX of UART
write_sentence_SC16(add, XHR, PMTK_SET_NMEA_OUTPUT_RMCONLY)

# write q release to RX of UART
write_sentence_SC16(add, XHR, PMTK_Q_RELEASE)

time.sleep(1)

write_log('log.log', add)

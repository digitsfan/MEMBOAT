#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
# pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
pwm = PWM(0x40, debug=True)

servozero = 4096*15/200  # Min pulse length out of 4096
servoninty = 4096*5/200  # Min pulse length out of 4096
servooneeighty = 4096*25/200 # Max pulse length out of 4096

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(50)                        # Set frequency to 60 Hz
try:
 while (True):
   # Change speed of continuous servo on channel O
   pwm.setPWM(0, 0, servozero)
   # pwm.setPWM(1, 0, servoMin)
   time.sleep(5)
   pwm.setPWM(0, 0, servoninty)
   time.sleep(5)
   pwm.setPWM(0, 0, servooneeighty)
   # pwm.setPWM(1, 0, servoMax)
   time.sleep(5)
except KeyboardInterrupt:
    pwm.setPWM(0, 0, servozero)



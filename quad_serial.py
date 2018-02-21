#
# Written by Roy Zohar, 2017.
# Published under the MIT license.
#

import cv2 as cv
import numpy as np
import serial       # pySerial

import logging

#
# Class for controlling drones through a computer.
# A special Arduino remote has to be connected for this to work.
# Either small arduina remote or with controller
#
class QuadSerial(object):
    serials = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyACM1"]

    def __init__(self):
        try:
            self._serial = serial.Serial("COM4", 115200, timeout=0)
        except:
            print("Unable to open serial")
            self._serial = None

    def is_opened(self):
        return self._serial is not None

    # write to serial
    def write(self, cmnds):
        if self._serial is not None:
            self._serial.write([0xff] + cmnds + [0, 0, 0])

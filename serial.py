#
# Class for controlling drones through a computer.
# A special Arduino remote has to be connected for this to work.
# Either small arduina remote or with controller
#

import serial       # pySerial

class QuadSerial(object):
    serials = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyACM1"]

    def __init__(self):
        try:
            self._serial = serial.Serial("COM4", 115200, timeout=0)
        except:
            print("Unable to open serial")
            self._serial = None

    def close():
        return os.close(self._serialfd)

    # write to serial
    def write(self, cmnds):
        if self._serial is not None
            self._serial.write([0xff] + cmnds + [0, 0, 0])
    

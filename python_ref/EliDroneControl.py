import cv2
import serial
import numpy as np

from collections import namedtuple

def nothing(x):
    pass

def createNormRemote():
    cv2.namedWindow('remote');

    cv2.createTrackbar('Right','remote', 100, 200, nothing)
    cv2.createTrackbar('Forward','remote', 100, 200, nothing)
    cv2.createTrackbar('Up','remote', 0, 200, nothing)
    cv2.createTrackbar('Clockwise','remote', 100, 200, nothing)
    return

def trySerialConnection(portName):
    try:
       ser = serial.Serial(portName, 115200, timeout=0)
       return ser
    except:
       return None

ChannelBounds = namedtuple('ChannelBounds', 'min zero max')

## Linear interpolatoin of a point
## Exists in Pytohn, simpler this way.

def lerp(t, a, b):
    return round((1 - t) * a + t * b)

def transformIntoChannel(normalizedValue, bounds):
    t = normalizedValue
    if t >= 0 :
        return lerp(t, bounds.zero, bounds.max)
    else :
        return lerp(-t, bounds.zero, bounds.min)

def clampControl(value, minVal, maxVal):
    if value > maxVal :
        return maxVal
    if value < minVal :
        return minVal
    return value

def toSignBitFormat(n):
    if n > 0:
        res = n
    else :
        res = -n
        res = res | 0x80

    return res
    
def sendToDrone(ser, normalizedControl):
    rightBounds = ChannelBounds(127 - 30, 64, 0 + 30)
    upBounds = ChannelBounds(10, 50, 100)
    forwardBounds = ChannelBounds(0 + 30, 64, 127 - 30)
    clockwiseBounds = ChannelBounds(127, 64, 0)

    right = transformIntoChannel(normalizedControl[0], rightBounds)
    up = transformIntoChannel(normalizedControl[1], upBounds)
    forward = transformIntoChannel(normalizedControl[2], forwardBounds)
    clockwise = transformIntoChannel(normalizedControl[3], clockwiseBounds)

    right = (right - 64) * 2
    up = (up - 64) * 2
    forward = (forward - 64) * 2
    clockwise = (clockwise - 64) * 2

    right = clampControl(right, -127, 127)
    up = clampControl(up, -127, 127)
    forward = clampControl(forward, -127, 127)
    clockwise = clampControl(clockwise, -127, 127)

    right = toSignBitFormat(right)
    up = toSignBitFormat(up)
    forward = toSignBitFormat(forward)
    clockwise = toSignBitFormat(clockwise)

    print(str(right) + ' ' + str(up) + ' ' + str(forward) + ' ' + str(clockwise))

    if ser is not None:
        control = [0xff, right, forward, up, clockwise, 0, 0, 0]
        ser.write(control)
        return
    else :
        print('Couldn\'t write to serial')
        return










cap = cv2.VideoCapture(2)
ser = trySerialConnection('COM4')

createNormRemote()

while(True):
    ret, frame = cap.read()
    
    right = (cv2.getTrackbarPos('Right', 'remote') - 100) / 100
    up = (cv2.getTrackbarPos('Up', 'remote') - 100) / 100
    forward = (cv2.getTrackbarPos('Forward', 'remote') - 100) / 100
    clockwise = (cv2.getTrackbarPos('Clockwise', 'remote') - 100) / 100

    normalizedControl = [right, up, forward, clockwise]
    sendToDrone(ser, normalizedControl)

#    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

normalizedControl = [right, -1, forward, clockwise]
sendToDrone(ser, normalizedControl)

cap.release()
if ser is not None:
    ser.close()
cv2.destroyAllWindows()


#
# Written by Roy Zohar, 2017.
# Published under the MIT license.
#

import cv2 as cv
import numpy as np

import logging

class Pid(object):
    def __init__(self, ki=0, kp=0, kd=0, minIntegral=-1, maxIntegral):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._lastValue = 0
        self._scaledErrorSum = 0
        self._minIntegral = minIntegral
        self._maxIntegral = maxIntegral

    # clamps the value between maxIntegral and minIntegral
    def clamp(self, value):
        if value > maxIntegral:
            return maxIntegral
        elif value < minIntegral:
            return minIntegral
        return value

    # calculates the pid (algorithm can be found in wikipedia)
    def calc_with_vals(self, currentValue, setPoint, deltaTime):
        return self.calculate(setPoint - currentValue, deltaTime)

    # calculates based on error from true value
    def calculate(self, error, deltaTime):
        # calculate error
        self._scaledErrorSum += self._ki * (error * deltaTime)
        self._scaledErrorSum = self.clamp(self._scaledErrorSum)

        # calculate errorDerivative
        errorDerivative = -(currentValue - lastValue) / deltaTime
        lastValue = currentValue

        # return the clamped pid
        return self._clamp(
            self._kp * error
            + self._scaledErrorSum
            + self._kd * errorDerivative
        )

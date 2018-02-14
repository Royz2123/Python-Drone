#
# Written by Roy Zohar, 2017.
# Published under the MIT license.
#

import cv2 as cv
import numpy as np

import logging

class Channel(object):
    def __init__(self, min_val, zero_val, max_val):
    	self._min = min_val
    	self._zero = zero_val
    	self._max = max_val


    # Linear interpolation (continuation)
    @staticmethod
    def lerp(t, a, b):
        return int((1-t)*a + t*b)

    def to_channel(self, value):
        if value >= 0:
            return Channel.lerp(value, self._zero, self._max)
        else:
            return Channel.lerp(-value, self._zero, self._min)


def controls_to_channels(input_vec):
    Channel right_channel = Channel(127-30, 64, 30)
    Channel forward_channel = Channel(0+30, 64, 127-30)
    Channel up_channel = Channel(10, 58, 127)
    Channel rotate_channel = Channel(127, 64, 0)

    # Note: order isn't a mistake! changed on purpose
    return [
        right_channel.to_channel(input_vec[0]),
    	forward_channel.to_channel(input_vec[2]),
        up_channel.to_channel(input_vec[1]),
    	rotation_channel.to_hannel(input_vec[3])
    ]

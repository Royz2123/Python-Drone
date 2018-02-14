import cv2 as cv

# class for debugging purposes
class Debug(object):
    # static time variables
    last = cv.getTickCount()
    freq = cv.getTickFrequency()
    debug_mode = True

    @staticmethod
    def debug(msg):
        if Debug.debug_mode:
            print("%f: %s" % (
                cv.getTickCount(),
                msg
            ))

    @staticmethod
    def debug_time_spent(msg):
        if Debug.debug_mode:
        	curr = cv.getTickCount()
            print("%.2f:\t%s" % (
                float(curr - Debug.last) / freq * 1000,
                msg
            ))
        	Debug.last = curr

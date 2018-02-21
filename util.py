import numpy as np


def nothing(x):
    pass


def is_number(s):
    try:
        int(s)
        return True
    except ValueError:
        return False



def calculate_control_errors(currPos, currRotation, targetPos):
    # yawCW = rotationYaw(currRotation)

    yawRotation  = [0, 1, 0] # TODO: Add yaw * -yawCW}

    droneWorldForward = yawRotation * np.array([0, 0, -1])
    droneWorldRight = yawRotation * np.array([1, 0, 0])
    droneWorldUp = yawRotation * np.array([0, 1, 0])

    target = targetPos - currPos

    errors = [
        target.dot(droneWorldRight),
        target.dot(droneWorldUp),
        target.dot(droneWorldForward),
        0   # -yawCW
    ]
    return errors

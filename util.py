def is_number(s):
    try:
        int(s)
        return True
    except ValueError:
        return False



def calculate_control_errors(currPos, currRotation, targetPos):
	# yawCW = rotationYaw(currRotation)

	Affine3f yawRotation{Vec3f{0, 1, 0} * -yawCW}

	droneWorldForward = yawRotation * [0, 0, -1]
	droneWorldRight = yawRotation * [1, 0, 0]
	droneWorldUp = yawRotation * [0, 1, 0]

	target = targetPos - currPos

	errors = [
        target.dot(droneWorldRight),
		target.dot(droneWorldUp),
		target.dot(droneWorldForward),
		0 - yawCW
    ]
	return errors

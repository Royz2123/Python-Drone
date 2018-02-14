#
# Written by Roy Zohar, 2017.
# Published under the MIT license.
#

import cv2 as cv
import numpy as np

import channels
import logging
import pid
import quad_serial
import image_processing
import viz

# Raspberry Camera module, not completely necessary
# include <raspicam/raspicam_cv.h>

# important keys
ARROW_LEFT = 65361
ARROW_UP = 65362
ARROW_RIGHT = 65363
ARROW_DOWN = 65364
BACK_BUTTON = 269025062
HOME_BUTTON = 269025048
ENTER = 13
PI_ENTER = 141
PI_BACK_BUTTON = 38

# camera constants
SONY_CAMERA_MAT = np.array([
    5.38614e+02, 0., 3.10130e+02,
    0., 5.38112e+02, 2.27066e+02,
    0., 0., 1.
]).reshape(3, 3)
TX_CAMERA_MAT = np.array([
    6.7628576774457656e+02, 0., 3.0519865395809290e+02,
    0., 6.7561534030641053e+02, 2.4692172053127743e+02,
    0., 0., 1.
]).reshape(3, 3)
PI_CAMERA_MAT = np.array([
    297.8761532662123, 0., 158.6603882333692,
    0., 296.2831432511588, 120.2546307355579,
    0., 0., 1.
]).reshape(3, 3)

INV_DRONE_CAMERA_TRANSFORM = np.array([cv.CV_PI/2, 0, 0]).inv()

# world constants
WORLD_SQUARE_SIZE = 21.0
WORLD_SQUARE = [
    [0, 0, 0],
    [0, 0, -WORLD_SQUARE_SIZE],
    [WORLD_SQUARE_SIZE, 0, WORLD_SQUARE_SIZE],
    [WORLD_SQUARE_SIZE, 0, 0]
]
DRONE_TARGET = [WORLD_SQUARE_SIZE, 160, -WORLD_SQUARE_SIZE]


# DEBUGGING constants
#
# viz_mode is cloning frames rapidly,
# Note: Heavy for the raspberry.
#
viz.GUI.viz_mode = True
viz.GUI.config_mode = True

# debug_mode logs everything
logging.Debug.debug_mode = True


def rotationYaw(rotation):
	zx = rotation.at<float>(0, 2)
	zz = rotation.at<float>(2, 2)
	atanY = zx
	atanX = zz

	yawCCW = std.atan2(atanY, atanX)

	return -yawCCW


def calculateControlErrors(currPos, currRotation, targetPos):
	yawCW = rotationYaw(currRotation)

	Affine3f yawRotation{Vec3f{0, 1, 0} * -yawCW}

	droneWorldForward = yawRotation * Vec3f{0, 0, -1}
	droneWorldRight = yawRotation * Vec3f{1, 0, 0}
	droneWorldUp = yawRotation * Vec3f{0, 1, 0}

	target = targetPos - currPos

	errors = [
        target.dot(droneWorldRight),
		target.dot(droneWorldUp),
		target.dot(droneWorldForward),
		0 - yawCW
    ]
	return errors


def is_number(s):
    try:
        int(s)
        return True
    except ValueError:
        return False


def main():
	#                                                                             #####
	# In Pattern mode, the drone stabilizes above a black pattern that looks like #   #
	#                                                                             #   #

	#
	# TX is the wireless camera we were using, and which required special handling:
	# In TX mode the image is flipped vertically, a small black area near
	# the borders is removed, and simple de-interlacing is applied.
	#
    txMode = False

	#
	# DEVO mode is for the older remotes that use a physical Walkera DEVO
	# drone controller. Leave this off of the small new remotes.
	#
	devoMode = False

	#
	# If we are running on a respberry computer,
	# we have the camera interface to grab photos from the RaspberryCam.
	#
	berryMode = False

	#
	# Images on RaspberryCam are too big,
	# we need to scale them down.
	#
	scaleFactor = 1.0

    # create a quad serial object
	serial = quad_serial.QuadSerial()

	pid_controllers = [
		pid.Pid(0.04, 0, 0),
		pid.Pid(0.06, 0, 0),
		pid.Pid(0.04, 0, 0),
		pid.Pid(0.3, 0, 0),
	]
    coeffs = {
        "pid" : {
            "xz" : {"p" : 4, "i" : 10, "d" : 25},
            "y" : {"p" : 5, "i" : 10, "d" : 33},
            "r" : {"p" : 30, "i" : 0, "d" : 0}
        },
        "smoothing" : 40
    }

	cameraIndex = 0

    # handle arguments
	for arg in sys.argv[1:]:
		if arg == "-ngui":
			viz.GUI.viz_mode = False
		elif temp == "-pi":
			berryMode = True
		elif temp == "-nconfig":
			viz.GUI.config_mode = False
		elif is_number(temp):
            cameraIndex = int(temp)

    viz.GUI.create_trackbars()

	cap = cv.VideoCapture(cameraIndex)

	frame = np.array([])
	camera_matrix = PI_CAMERA_MAT

	cameraSquare = np.zeros(3)
    drone_transform = np.zeros(3)
	smoothPosition = np.zeros(3)

	last_frame_tick_count = 0

	flightModeOn = False
	paused = False
	pressedKey = 0

	while pressedKey != 'q':
		logging.Debug.debug("************** Entered Loop *****************")
		if not paused:
            ret, frame = cap.read()
            if ret is None:
                break

			logging.Debug.debug("Photo grabing")

			if scaleFactor != 1.0:
				resize(frame, frame, Size(), scaleFactor, scaleFactor, cv.INTER_NEAREST)

			logging.Debug.debug("Resizing")

            # TX image processing
			if txMode:
				frame = frame[5:, :-7]
				for i in range(0, frame.rows - 1, 2):
					frame.row(i + 1).copyTo(frame.row(i))
				cv.flip(frame, frame, -1)

			logging.Debug.debug("TX Mode")

		deltaTime = float(cv.getTickCount() - last_frame_tick_count) / cv.getTickFrequency()
		last_frame_tick_count = cv.getTickCount()

		logging.Debug.debug("Time since last frame tick count")

		camera_square = ImageProcessing.find_open_square(frame)
        viz.GUI.draw_square(frame, cameraSquare)

		logging.Debug.debug("Tracking OpenSquare section")

		if camera_square is None:
		    logging.Debug.debug("Square not found")
        else:
			rvec, tvec = cv.solvePnP(
                WORLD_SQUARE,
				cameraSquare,
				camera_matrix,
			)

			logging.Debug.debug("SolvePnP")

			rvec.convertTo(rvec, CV_32F)
			tvec.convertTo(tvec, CV_32F)
			Affine3f camera_transform = Affine3f{rvec, tvec}
			# The square doesn't move, we are moving.
			camera_transform = camera_transform.inv()

			# We found the camera, but we want to find the drone
			drone_transform = camera_transform * INV_DRONE_CAMERA_TRANSFORM

			pos = drone_transformform.translation()
			smoothPosition = coeffs["smoothing"] * smoothPosition + (1 - coeffs["smoothing"]) * pos

			drone_transform.translation(smoothPosition)

        logging.Debug.debug("found Position")

        # update controller coefficients based on trackbar
        for index, axis in enumerate(["xz", "y", "xz", "r"]):
            factor = 1000.0 if (axis != "r") else 100.0
    		pid_controllers[index]._kp = coeffs[axis]["p"] / factor
    		pid_controllers[index]._ki = coeffs[axis]["i"] / (factor * 10)
    		pid_controllers[index]._kd = coeffs[axis]["d"] / (factor * 10)

		controlErrors = calculateControlErrors(
            drone_transform.translation(),
            drone_transform.rotation(),
            DRONE_TARGET
        )

		logging.Debug.debug("PID")

        # set pid controls
        pid_control = [0, 0, 0, 0]
		if found and flightModeOn:
			for i in range(4):
				pid_control.append(pid_controllers[i].calculate(controlErrors[i], deltaTime))

		logging.Debug.debug("Take off and Landing")

		channel_controls = chnnels.controls_to_drone_channels(pid_control)
		if !flightModeOn:
			channel_controls = [64, 64, 0, 64]

		if serial.is_opened():
			success = serial.send([
				(int(channel_controls[0]) - 64) * 2,
				(int(channel_controls[1]) - 64) * 2,
				(int(channel_controls[2]) - 64) * 2,
				(int(channel_controls[3]) - 64) * 2
            ])

			if not success:
                logging.Debug.debug("Failed to send to Arduino")

		logging.Debug.debug("Control errors to console")

		# Draw GUI
		viz.GUI.drawFlightViz(
            displayedFrame,
			displayedFrame,
			drone_transform,
			59.9,
			droneTarget,
			pidControl * 100,
			flightModeOn
        )

		logging.Debug.debug("Draw GUI")

		pressedKey = cv.waitKey(1)
        
		logging.Debug.debug("cv.waitKey(1)")

		if pressedKey == ' ' :
			paused = !paused
        elif pressedKey in [ENTER. PI_ENTER, 'x']:
			flightModeOn = !flightModeOn
        elif pressedKey in [BACK_BUTTON, PI_BACK_BUTTON, 'i']:
			for pid in pid_controllers:
				pid._scaled_error_sum = 0
        elif pressedKey in ['q']:
            break
        elif pressedKey == '0':
            viz.stepViz.displayed_step = 9
        elif pressedKey >= '1' and pressedKey <= '9':
            vizstepViz.displayed_step = pressedKey - '1'

		logging.Debug.debug("User Commands")


if __name__ == "__main__":
    main()

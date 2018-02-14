#
# Written by Roy Zohar, 2017.
# Published under the MIT license.
#

import os
import cv2 as cv


# Raspberry Camera module, not completely necessary
# include <raspicam/raspicam_cv.h>


import pid
import quad_serial

# frequent colors
RED = np.array([66, 66, 244])
GREEN = np.array([66, 244, 66])

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


#define QQQ do {std.cerr << "QQQ " << __FUNCTION__ << " " << __LINE__ << std.endl} while(0)

#define VIZ_STEP if (++stepViz.currentStep == stepViz.displayed_step && VIZStepFunction)

#define StepVizData.vizMat stepViz.vizMat

#
# viz_mode is cloning frames rapidly,
# Note: Heavy for the raspberry.
#
viz_mode = True

# debug_mode logs everything
debug_mode = True







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


def trackbar_action(value, coeffs, name):
    if name == "smoothing":
        # update the somothing factor
        coeffs["smoothing"] = value
    else:
        # update the trackbars for the pid
        axis, pid_name = name.split(' ')
        coeffs["pid"][axis][pid_name] = value


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

	#
	# Sometimes we don't want to create this window.
	#
	config_mode = True

    # create a quad serial object
	quad_serial.QuadSerial serial
	serial.openDefault()

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

	for arg in sys.argv[1:]:
		if arg == "-ngui":
			viz_mode = False
		elif temp == "-pi":
			berryMode = True
		elif temp == "-nconfig":
			config_mode = False
		elif is_number(temp):
            cameraIndex = int(temp)

	if config_mode:
		cv.namedWindow("config", cv.WINDOW_NORMAL)

        # create trackbars for pid
        for axis_name, pid in coeffs["pid"].items():
            for pid_name, pid_val in pid.items():
                trackbar_name = "%s %s" % (axis, pid_name)
                cv.createTrackbar(
                    trackbar_name,
                    "config",
                    curr_val,
                    100,
                    lambda x: trackbar_action(x, coeffs, trackbar_name)
                )

        # create trackbar for smoothing factor
		cv.createTrackbar(
            "Smoothing factor",
            "config",
            coeffs["smoothing"],
            100,
            lambda x: trackbar_action(x, coeffs, "smoothing")
        )
	}

	raspicam.RaspiCam_Cv piCap
	VideoCapture cap

	if berryMode:
		logging.Debug.debug("Opening Camera")
		piCap.set( CV_CAP_PROP_FORMAT, CV_8UC1 )
		piCap.set( CV_CAP_PROP_FRAME_WIDTH, 320 )
		piCap.set( CV_CAP_PROP_FRAME_HEIGHT, 240 )

		if not piCap.open():
            logging.Debug.debug("Error opening the camera")
			return -1
	else:
		cap.open(cameraIndex)

		if not cap.isOpened():
            logging.Debug.debug("Error opening the camera: %d" % cameraIndex)
			return -1

		cap.set(cv.CAP_PROP_FPS, 60)

	frame = np.array([])
	camera_matrix = PI_CAMERA_MAT

	cameraSquare = np.zeros(3)
    droneTransform = np.zeros(3)
	smoothPosition = np.zeros(3)

	last_frame_tick_count = 0

	flightModeOn = False
	paused = False
	pressedKey = 0

	while (pressedKey != 'q') {
		logging.Debug.debug("************** Entered Loop *****************")
		if (!paused) {
			if(berryMode):
				piCap.grab()
				piCap.retrieve(frame)
			else:
				cap >> frame

			logging.Debug.debug("Photo grabing")

			if scaleFactor != 1.0:
				resize(frame, frame, Size(), scaleFactor, scaleFactor, cv.INTER_NEAREST)

			logging.Debug.debug("Resizing")

			if txMode:

				#
				# Deinterlace and flip
				#

				# TODO: Do solve PnP in the original frame

				frame = frame(Range{5, frame.rows}, Range{0, frame.cols - 7})

				# NOTE(Andrey): From experiments we found the odd lines are
				# more recent in both TX05 and TX03.

				for i in range(0, frame.rows - 1, 2):
					frame.row(i + 1).copyTo(frame.row(i))

				const int horizontalAndVertical = -1
				cv.flip(frame, frame, horizontalAndVertical)

			logging.Debug.debug("TX Mode")

        if viz_mode:
            stepViz = frame.clone()

		logging.Debug.debug("StepVizData.vizMat clone")

		deltaTime = float(cv.getTickCount() - last_frame_tick_count) / cv.getTickFrequency()
		last_frame_tick_count = cv.getTickCount()

		logging.Debug.debug("Time since last frame tick count")

		found = ImageProcessing.find_open_square(frame, cameraSquare)

		if viz_mode:
			StepVizData.vizMat = frame.clone()
			for i in range(cameraSquare.size()){
				cv.line(
                    StepVizData.vizMat,
                    cameraSquare[i],
                    cameraSquare[(i + 1) % cameraSquare.size()],
                    RED if (i == cameraSquare.size() - 1) else GREEN,
                    3
                )
			}
		}

		logging.Debug.debug("Tracking OpenSquare section")

		if not found:
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
			Affine3f cameraTransform = Affine3f{rvec, tvec}
			# The square doesn't move, we are moving.
			cameraTransform = cameraTransform.inv()

			# We found the camera, but we want to find the drone
			droneTransform = cameraTransform * invDroneCameraTransform

			pos = droneTransform.translation()
			smoothPosition = coeffs["smoothing"] * smoothPosition + (1 - coeffs["smoothing"]) * pos

			droneTransform.translation(smoothPosition)

        logging.Debug.debug("found Position")

        # update controller coefficients based on trackbar
        for index, axis in enumerate(["xz", "y", "xz", "r"]):
            factor = 1000.0 if (axis != "r") else 100.0
    		pid_controllers[index]._kp = coeffs[axis]["p"] / factor
    		pid_controllers[index]._ki = coeffs[axis]["i"] / (factor * 10)
    		pid_controllers[index]._kd = coeffs[axis]["d"] / (factor * 10)

		controlErrors = calculateControlErrors(
            droneTransform.translation(),
            Mat{droneTransform.rotation()},
            DRONE_TARGET
        )

		logging.Debug.debug("PID")

        # set pid controls
        pid_control = [0, 0, 0, 0]
		if found and flightModeOn:
			for i in range(4):
				pid_control.append(pid_controllers[i].calculate(controlErrors[i], deltaTime))

		logging.Debug.debug("Take off and Landing")

		channel_controls = controls_to_drone_channels(pid_control)
		if !flightModeOn:
			channel_controls = [64, 64, 0, 64]

		if serial.isOpened():
			# TODO(Andrey): Work with floats instead
			if devoMode:
				success = serial.sendDevo(
					channel_controls[0],
					channel_controls[1],
					channel_controls[2],
					channel_controls[3]
                )
			else:
				success = serial.send(
					(int(channel_controls[0]) - 64) * 2,
					(int(channel_controls[1]) - 64) * 2,
					(int(channel_controls[2]) - 64) * 2,
					(int(channel_controls[3]) - 64) * 2
                )

			if not success:
                logging.Debug.debug("Failed to send to Arduino")
			logging.Debug.debug("Send drone actions")
		}


		logging.Debug.debug("Control errors to console")

		#
		# Draw GUI
		#
        if viz_mode:
			displayed_frame = StepVizData.vizMat.clone()

			if displayedFrame.empty():
				displayedFrame = Mat{32, 32, CV_8UC3}
			elif displayedFrame.type() == CV_8U:
				cv.cvtColor(displayedFrame, displayedFrame, cv.COLOR_GRAY2BGR)

			GUI.drawFlightViz(displayedFrame,
				displayedFrame,
				droneTransform,
				59.9,
				droneTarget,
				pidControl * 100,
				flightModeOn
            )

			cv.imshow("w", displayedFrame)
			showFrameRateInTitle("w")
		logging.Debug.debug("Draw GUI")

		pressedKey = cv.waitKey(1)

		#std.cout << pressedKey << std.endl

		logging.Debug.debug("cv.waitKey(1)")

		if pressedKey == ' ' :
			paused = !paused
        elif pressedKey in [ENTER. PI_ENTER, 'x']:
			flightModeOn = !flightModeOn
        elif pressedKey in [BACK_BUTTON, PI_BACK_BUTTON, 'i']:
			for pid in pid_controllers:
				pid._scaled_error_sum = 0

		logging.Debug.debug("User Commands")

		#
		# Prepare StepViz for next cycle
		#
        if viz_mode:
			# stepViz.vizMat.setTo(0xCC)

			if pressedKey == '0':
				stepViz.displayed_step = 9
			elif pressedKey >= '1' and pressedKey <= '9':
				stepViz.displayed_step = pressedKey - '1'
	return 0
}




if __name__ == "__main__":
    main()

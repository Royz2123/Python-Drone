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

class StepVizData(object):
    displayed_step = 0
    current_step = -1
    vizMat                                    # TODO: define as a cv.Mat (np.array)

class GUI(object):
    def __init__(self):
        # fps calculations
        self._frames = 0                        # number of frames
        self._start = cv.getTickCount()         # set start tickcount
        self._freq = cv.getTickFrequency()      # set tick frequency
        self._capture_length = self._freq / 10

        self._main_window = ""

    def show_frame_rate(self):
    	self._freq = static_cast<int>(cv.getTickFrequency())
    	self._captureLength = self._freq / 10

        # move onto next frame
    	self._frames++

        # current tick count
    	curr = cv.getTickCount()

    	if (curr - start) >= captureLength:
    		fps = frames * (freq / (curr - start))
            if VIZStepFunction:
    			cv.setWindowTitle(window, str(fps))
            print("FPS:\t" % fps)
    		self._start = curr
            self._frames = 0

    def ortho_project(point, camera_matrix):
    	point = camera_matrix * point
    	return Point2i{(int)point.x, (int)point.z}

    def drawViewOrthographicXZ(
		screenBuffer,
		size,
		name,
		droneTransform,
		droneTarget,
		droneControlErrors,
		transform,
		drawPlaneIcon=False
    ):
    	screenBuffer.create(size.height, size.width, CV_8UC3)
    	screenBuffer.setTo(Scalar{51, 51, 51})

    	Point3f xBegin{-100 * 1000, 0, 0}
    	Point3f xEnd = -xBegin
    	Point3f yBegin{0, -100 * 1000, 0}
    	Point3f yEnd = -yBegin
    	Point3f zBegin{0, 0, -100 * 1000}
    	Point3f zEnd = -zBegin

    	Scalar axesColor{242, 158, 106}

    	cv.line(screenBuffer, orthoProject(xBegin, transform), orthoProject(xEnd, transform), axesColor)
    	cv.line(screenBuffer, orthoProject(yBegin, transform), orthoProject(yEnd, transform), axesColor)
    	cv.line(screenBuffer, orthoProject(zBegin, transform), orthoProject(zEnd, transform), axesColor)

    	Point2i planePosition = orthoProject(droneTransform.translation(), transform)
    	cv.circle(screenBuffer, planePosition, 3, Scalar{255, 255, 255}, 2)


    	cv.putText(screenBuffer, name, {size.width - 50, 27}, cv.FONT_HERSHEY_PLAIN, 1, axesColor)

    	if (drawPlaneIcon) {
    		Mat currRotation{droneTransform.rotation()}

    		float planeRotation = rotationYaw(Mat{droneTransform.rotation()})

    		# http:#www.challengers101.com/images/CompRose1.jpg
    		vector<Point2i> planeIconHalf1 = {
    			{0,  -141},
    			{0,  -108},
    			{9,  -90},
    			{12, -71},
    			{14, -53},
    			{75, 6},
    			{76, 13},
    			{75, 26},
    			{12, -6},
    			{9,  49},
    			{30, 71},
    			{33, 80},
    			{31, 89},
    			{4,  76},
    			{0,  82},
    		}

    		vector<Point2i> planeIconHalf2 = planeIconHalf1
    		std.reverse(planeIconHalf2.begin(), planeIconHalf2.end())

    		for (auto &p : planeIconHalf2) {
    			p = Point2i{-p.x, p.y}
    		}

    		vector<Point2i> planeIcon = planeIconHalf1
    		planeIcon.insert(planeIcon.begin(), planeIconHalf2.begin(), planeIconHalf2.end())

    		for (auto &p : planeIcon) {
    			float cosT = std.cos(planeRotation)
    			float sinT = std.sin(planeRotation)
    			p = Point2i{(int)(p.x * cosT - p.y * sinT), (int)(p.x * sinT + p.y * cosT)}

    			p /= 2
    			p += planePosition
    		}

    		vector<vector<Point2i>> planeIconWrapper = {planeIcon}

    		cv.polylines(screenBuffer, planeIconWrapper, False, Scalar{255, 255, 255})
    	}

    	Point2i droneTargetPosition = orthoProject(droneTarget, transform)
    	int rectSide = 10
    	cv.rectangle(screenBuffer, Rect2i{droneTargetPosition.x - rectSide / 2, droneTargetPosition.y - rectSide / 2, rectSide, rectSide}, Scalar{66, 66, 244}, 2)

    	Point3f controlErrors{droneControlErrors[0], droneControlErrors[1], -droneControlErrors[2]}
    	#controlErrors2D = droneTransform * controlErrors

    	Point2i dronePosition = orthoProject(droneTransform.translation(), transform)
    	Point2i shiftedControlErrors = orthoProject(droneTransform * controlErrors, transform)
    	cv.arrowedLine(screenBuffer, dronePosition, shiftedControlErrors, Scalar{66, 66, 244}, 2)
    }

    def drawFlightViz(
        screenBuffer,
		cameraFeed,
		droneTransform,
		frameRate,
		droneTarget,
		droneControlErrors,
		flightModeOn
    ):
    	Mat q1, q2, q3, q4

    	q1 = cameraFeed

    	Rect2i panelSize{0, 0, q1.cols, q1.rows}

    	Mat rotationTop = (cv.Mat_<float>(3, 3) <<
    		1, 0, 0,
    		0, 1, 0,
    		0, 0, 1)
    	float scaleTop = 1
    	Vec3f shiftTop{(float)panelSize.width / 2, 0, (float)panelSize.height / 2}

    	Mat rotationBack = (cv.Mat_<float>(3, 3) <<
    		1, 0, 0,
    		0, 0, 1,
    		0, -1, 0)
    	float scaleBack = 1
    	Vec3f shiftBack{(float)panelSize.width / 2, 0, (float)panelSize.height * 3 / 4}

    	Mat rotationLeft = (cv.Mat_<float>(3, 3) <<
    		0, 0, 1,
    		1, 0, 0,
    		0, -1, 0)
    	float scaleLeft = 1
    	Vec3f shiftLeft{(float)panelSize.width / 2, 0, (float)panelSize.height * 3 / 4}

    	drawViewOrthographicXZ(q2, panelSize.size(), "TOP" , droneTransform, DRONE_TARGET, droneControlErrors, Affine3f{rotationTop * scaleTop, shiftTop}, True)
    	drawViewOrthographicXZ(q3, panelSize.size(), "LEFT", droneTransform, DRONE_TARGET, droneControlErrors, Affine3f{rotationLeft * scaleLeft, shiftLeft})
    	drawViewOrthographicXZ(q4, panelSize.size(), "BACK", droneTransform, DRONE_TARGET, droneControlErrors, Affine3f{rotationBack * scaleBack, shiftBack})

    	if (!flightModeOn) {
    		auto text = "Flight Mode OFF. Press X to begin flight."
    		auto font = cv.FONT_HERSHEY_SIMPLEX
    		auto fontScale = 0.65
    		auto thickness = 2

    		int tmpBaseline
    		Size textSize = cv.getTextSize(text, font, fontScale, thickness, &tmpBaseline)

    		cv.putText(q2,
    				text, Point{(q2.cols - textSize.width) / 2, 20 + textSize.height},
    				font, fontScale, Scalar{0, 0, 255}, thickness)
    	}

    	Size2i totalSize{panelSize.width * 2 + 1, panelSize.height * 2 + 1}

    	screenBuffer.create(totalSize, CV_8UC3)
    	screenBuffer.setTo(Scalar{255, 255, 255})

    	q1.copyTo(Mat{screenBuffer, Rect2i{0, 0, panelSize.width, panelSize.height}})
    	q2.copyTo(Mat{screenBuffer, Rect2i{panelSize.width + 1, 0, panelSize.width, panelSize.height}})
    	q3.copyTo(Mat{screenBuffer, Rect2i{0, panelSize.height + 1, panelSize.width, panelSize.height}})
    	q4.copyTo(Mat{screenBuffer, Rect2i{panelSize.width + 1, panelSize.height + 1, panelSize.width, panelSize.height}})
    }



# class for debugging purposes
class Debug(object):
    # static time variables
    last = cv.getTickCount()
    freq = cv.getTickFrequency()

    @staticmethod
    def debug(msg):
        if debug_mode:
            print("%f: %s" % (
                cv.getTickCount(),
                masg
            ))

    @staticmethod
    def debug_time_spent(msg):
        if debug_mode:
        	curr = cv.getTickCount()
            print("%.2f:\t%s" % (
                float(curr - last) / freq * 1000,
                msg
            ))
        	last = curr


# Image processing techniques
# relies heavily on OpenCV
class ImageProcessing(object):
    @staticmethod
    def binarizeImageInv(src, dst):
    	Debug.debug("Entered binarizeImageInv")

    	dst = src.clone()

    	if dst.type() == cv.CV_8UC3:
    		cv.cvtColor(dst, dst, CV_BGR2GRAY)

    	CV_Assert(image.type() == CV_8U)

    	Debug.debug("Clone, convert and assert")

    	#cv.blur(image, image, Size{4, 4})

    	Debug.debug("Blur")

    	#double minColor
    	#cv.minMaxIdx(image, &minColor)
    	#cv.threshold(image, image, minColor + 50, 255, cv.CV_THRESH_BINARY_INV)

    	meanColor = cv.mean(dst)[0]
    	cv.threshold(dst, dst, meanColor - 30, 255, cv.CV_THRESH_BINARY_INV)

    	Debug.debug("Threshold")

    	#cv.threshold(image, image, 0, 255, CV_THRESH_OTSU)
    	#cv.bitwise_not(image, image)

    	#cv.morphologyEx(image,
    	#		image,
    	#		cv.MORPH_CLOSE,
    	#		cv.getStructuringElement(cv.MORPH_ELLIPSE, Size{7, 7}))

    	Debug.debug("morphologyEx close")

    	#cv.morphologyEx(image,
    	#		image,
    	#		cv.MORPH_OPEN,
    	#		cv.getStructuringElement(cv.MORPH_ELLIPSE, Size{7, 7}))

    	Debug.debug("morphologyEx open")

        if viz_mode:
            stepViz = dstc.clone()

    	Debug.debug("Finish binarizeImageInv")

    #                     ####
    # Finds the pattern:  #  #  in the image.
    #                    #  #
    def find_open_square(image, square):
    	Debug.debug("Entered findOpenSquare")
    	square.resize(0)

    	binarizeImageInv(image, image)

    	#
    	# Find the contours
    	#

    	vector<vector<Point2i>> contours

    	{
    		Mat imageCopy = image.clone()

    		cv.findContours(imageCopy,
    				contours,
    				cv.noArray(),
    				CV_RETR_LIST,
    				CV_CHAIN_APPROX_NONE)
    	}

    	Debug.debug("Find conturs")

    	if (contours.size() == 0) {
    		return False
    	}

    	#
    	# Select contour with largest area
    	#

    	int largestContourIndex = -1

    	{
    		double maxArea = -1

    		for (int i = 0 i < (int)contours.size() i++) {
    			double currArea = cv.contourArea(contours[i])

    			if (currArea > maxArea) {
    				maxArea = currArea
    				largestContourIndex = i
    			}
    		}
    	}

    	Debug.debug("Largest Area")

    	vector<Point2i> chosenContour = contours[largestContourIndex]

    	VIZ_STEP
    	{
    		StepVizData.vizMat = Mat.zeros(image.size(), CV_8U)

    		cv.drawContours(StepVizData.vizMat, contours, largestContourIndex, 255, -1)
    	}

    	#
    	# Find bounding square
    	#

    	cv.RotatedRect boundingRect = cv.minAreaRect(chosenContour)
    	auto size = boundingRect.size
    	boundingRect.size.width = boundingRect.size.height = (size.width + size.height) / 2
    	square.resize(4)
    	boundingRect.points(&square[0])

    	Debug.debug("Find Bounding square")

    	#
    	# Reorder square points into CW order
    	#

    	Point2f centroid = (square[0] + square[1] + square[2] + square[3]) / 4

    	std.sort(square.begin(),
    			square.end(),
    			[centroid](const Point2f &p1, const Point2f &p2) {
    			Point2f v1 = p1 - centroid
    			Point2f v2 = p2 - centroid
    			return std.atan2(-v1.y, v1.x) > std.atan2(-v2.y, v2.x)
    			})

    	Debug.debug("Reorder points")

    	#
    	# Find the missing edge.
    	# The missing edge is the edge whose midpoint
    	# is farther from any contour point than the midpint
    	# of any other edge.
    	#

    	float maxMinDistance = -1
    	int maxMinDistanceIndex = -1

    	for (unsigned int i = 0 i < square.size() i++) {
    		Point2f edgeCenter = (square[i] + square[(i + 1) % square.size()]) / 2

    		float currMinDistance = FLT_MAX

    		for (unsigned int k = 0 k < chosenContour.size() k++) {
    			float dx = edgeCenter.x - chosenContour[k].x
    			float dy = edgeCenter.y - chosenContour[k].y

    			float sqrDist = dx * dx + dy * dy

    			currMinDistance = std.min(currMinDistance, sqrDist)
    		}

    		if (currMinDistance > maxMinDistance) {
    			maxMinDistance = currMinDistance
    			maxMinDistanceIndex = i
    		}
    	}

    	std.rotate(square.begin(), square.begin() + (maxMinDistanceIndex + 1) % square.size(), square.end())

    	Debug.debug("Find missing edge and finish findOpenSquare")

    	return True
    }




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
		Debug.debug("Opening Camera")
		piCap.set( CV_CAP_PROP_FORMAT, CV_8UC1 )
		piCap.set( CV_CAP_PROP_FRAME_WIDTH, 320 )
		piCap.set( CV_CAP_PROP_FRAME_HEIGHT, 240 )

		if not piCap.open():
            Debug.debug("Error opening the camera")
			return -1
	else:
		cap.open(cameraIndex)

		if not cap.isOpened():
            Debug.debug("Error opening the camera: %d" % cameraIndex)
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
		Debug.debug("************** Entered Loop *****************")
		if (!paused) {
			if(berryMode):
				piCap.grab()
				piCap.retrieve(frame)
			else:
				cap >> frame

			Debug.debug("Photo grabing")

			if scaleFactor != 1.0:
				resize(frame, frame, Size(), scaleFactor, scaleFactor, cv.INTER_NEAREST)

			Debug.debug("Resizing")

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

			Debug.debug("TX Mode")

        if viz_mode:
            stepViz = frame.clone()

		Debug.debug("StepVizData.vizMat clone")

		deltaTime = float(cv.getTickCount() - last_frame_tick_count) / cv.getTickFrequency()
		last_frame_tick_count = cv.getTickCount()

		Debug.debug("Time since last frame tick count")

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

		Debug.debug("Tracking OpenSquare section")

		if not found:
		    Debug.debug("Square not found")
        else:
			rvec, tvec = cv.solvePnP(
                WORLD_SQUARE,
				cameraSquare,
				camera_matrix,
			)

			Debug.debug("SolvePnP")

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

        Debug.debug("found Position")

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

		Debug.debug("PID")

        # set pid controls
        pid_control = [0, 0, 0, 0]
		if found and flightModeOn:
			for i in range(4):
				pid_control.append(pid_controllers[i].calculate(controlErrors[i], deltaTime))

		Debug.debug("Take off and Landing")

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
                Debug.debug("Failed to send to Arduino")
			Debug.debug("Send drone actions")
		}


		Debug.debug("Control errors to console")

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
		Debug.debug("Draw GUI")

		pressedKey = cv.waitKey(1)

		#std.cout << pressedKey << std.endl

		Debug.debug("cv.waitKey(1)")

		if pressedKey == ' ' :
			paused = !paused
        elif pressedKey in [ENTER. PI_ENTER, 'x']:
			flightModeOn = !flightModeOn
        elif pressedKey in [BACK_BUTTON, PI_BACK_BUTTON, 'i']:
			for pid in pid_controllers:
				pid._scaled_error_sum = 0

		Debug.debug("User Commands")

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

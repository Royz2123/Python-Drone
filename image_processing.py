# Image processing techniques
# relies heavily on OpenCV
class ImageProcessing(object):
    @staticmethod
    def binarizeImageInv(src, dst):
    	logging.Debug.debug("Entered binarizeImageInv")

    	dst = src.clone()

    	if dst.type() == cv.CV_8UC3:
    		cv.cvtColor(dst, dst, CV_BGR2GRAY)

    	CV_Assert(image.type() == CV_8U)

    	logging.Debug.debug("Clone, convert and assert")

    	#cv.blur(image, image, Size{4, 4})

    	logging.Debug.debug("Blur")

    	#double minColor
    	#cv.minMaxIdx(image, &minColor)
    	#cv.threshold(image, image, minColor + 50, 255, cv.CV_THRESH_BINARY_INV)

    	meanColor = cv.mean(dst)[0]
    	cv.threshold(dst, dst, meanColor - 30, 255, cv.CV_THRESH_BINARY_INV)

    	logging.Debug.debug("Threshold")

    	#cv.threshold(image, image, 0, 255, CV_THRESH_OTSU)
    	#cv.bitwise_not(image, image)

    	#cv.morphologyEx(image,
    	#		image,
    	#		cv.MORPH_CLOSE,
    	#		cv.getStructuringElement(cv.MORPH_ELLIPSE, Size{7, 7}))

    	logging.Debug.debug("morphologyEx close")

    	#cv.morphologyEx(image,
    	#		image,
    	#		cv.MORPH_OPEN,
    	#		cv.getStructuringElement(cv.MORPH_ELLIPSE, Size{7, 7}))

    	logging.Debug.debug("morphologyEx open")

        if viz_mode:
            stepViz = dstc.clone()

    	logging.Debug.debug("Finish binarizeImageInv")

    #                     ####
    # Finds the pattern:  #  #  in the image.
    #                    #  #
    def find_open_square(image, square):
    	logging.Debug.debug("Entered findOpenSquare")
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

    	logging.Debug.debug("Find conturs")

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

    	logging.Debug.debug("Largest Area")

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

    	logging.Debug.debug("Find Bounding square")

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

    	logging.Debug.debug("Reorder points")

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

    	logging.Debug.debug("Find missing edge and finish findOpenSquare")

    	return True
    }

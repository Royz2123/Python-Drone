//
// Written by Andrey Leshenko and Eli Tarnarutsky, 2017.
// Published under the MIT license.
//

#include "pid.hpp"
#include "quad_serial.hpp"

// Raspberry Camera
#include <raspicam/raspicam_cv.h>

#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

using std::vector;

using cv::Mat;
using cv::Scalar;
using cv::Point;
using cv::Point2f;
using cv::Point2i;
using cv::Point3f;
using cv::Vec3f;
using cv::Vec4f;
using cv::Vec4i;
using cv::Vec4b;
using cv::Size;
using cv::Size2i;
using cv::Range;
using cv::Rect2i;
using cv::Affine3d;
using cv::Affine3f;
using cv::VideoCapture;

#define QQQ do {std::cerr << "QQQ " << __FUNCTION__ << " " << __LINE__ << std::endl;} while(0)

#define VIZ_STEP if (++stepViz.currentStep == stepViz.displayedStep && VIZStepFunction)

#define VIZ_MAT stepViz.vizMat

//
// VIZStep is cloning frames rapidely,
// Heavy for the raspberry.
//
bool VIZStepFunction = true;

struct StepVizData
{
	int displayedStep = 0;
	int currentStep = -1;
	Mat vizMat;
} stepViz;

void showFrameRateInTitle(const char* window)
{
	static int64 freq = static_cast<int>(cv::getTickFrequency());
	static int64 captureLength = freq / 10;

	static int64 start = cv::getTickCount();
	static int frames = 0;

	frames++;

	int64 curr = cv::getTickCount();

	if ((curr - start) >= captureLength) {
		int fps = frames * (freq / (curr - start));
		if(VIZStepFunction)
			cv::setWindowTitle(window, std::to_string(fps));
		std::cout << fps << std::endl;
		start = curr;
		frames = 0;
	}
}

void printTimeSinceLastCall(const char* message)
{
	static int64 freq = static_cast<int>(cv::getTickFrequency());
	static int64 last = cv::getTickCount();

	int64 curr = cv::getTickCount();
	int64 delta = curr - last;
	double deltaMs = (double)delta / freq * 1000;
	// printf("%s: %.2f\n", message, deltaMs);

	last = curr;
}

void binarizeImageInv(const Mat &src, Mat &dst)
{
	printTimeSinceLastCall("Entered binarizeImageInv");
	
	Mat &image = dst;
	image = src.clone();

	if (image.type() == CV_8UC3) {
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	CV_Assert(image.type() == CV_8U);

	printTimeSinceLastCall("Clone, convert and assert");

	//cv::blur(image, image, Size{4, 4});

	printTimeSinceLastCall("Blur");

	//double minColor;
	//cv::minMaxIdx(image, &minColor);
	//cv::threshold(image, image, minColor + 50, 255, CV_THRESH_BINARY_INV);

	int meanColor = cv::mean(image)[0];
	cv::threshold(image, image, meanColor - 30, 255, CV_THRESH_BINARY_INV);

	printTimeSinceLastCall("Threshold");

	//cv::threshold(image, image, 0, 255, CV_THRESH_OTSU);
	//cv::bitwise_not(image, image);

	//cv::morphologyEx(image,
	//		image,
	//		cv::MORPH_CLOSE,
	//		cv::getStructuringElement(cv::MORPH_ELLIPSE, Size{7, 7}));

	printTimeSinceLastCall("morphologyEx close");

	//cv::morphologyEx(image,
	//		image,
	//		cv::MORPH_OPEN,
	//		cv::getStructuringElement(cv::MORPH_ELLIPSE, Size{7, 7}));

	printTimeSinceLastCall("morphologyEx open");

	VIZ_STEP
	{
		VIZ_MAT = image.clone();
	}
	printTimeSinceLastCall("Finish binarizeImageInv");
}

//                    ####
// Find the pattern:  #  #  in the image.
//                    #  #

bool findOpenSquare(Mat image, vector<Point2f> &square)
{
	printTimeSinceLastCall("Entered findOpenSquare");
	square.resize(0);

	binarizeImageInv(image, image);
	
	//
	// Find the contours
	//

	vector<vector<Point2i>> contours;

	{
		Mat imageCopy = image.clone();

		cv::findContours(imageCopy,
				contours,
				cv::noArray(),
				CV_RETR_LIST,
				CV_CHAIN_APPROX_NONE);
	}

	printTimeSinceLastCall("Find conturs");

	if (contours.size() == 0) {
		return false;
	}

	//
	// Select contour with largest area
	//

	int largestContourIndex = -1;

	{
		double maxArea = -1;

		for (int i = 0; i < (int)contours.size(); i++) {
			double currArea = cv::contourArea(contours[i]);

			if (currArea > maxArea) {
				maxArea = currArea;
				largestContourIndex = i;
			}
		}
	}

	printTimeSinceLastCall("Largest Area");

	vector<Point2i> chosenContour = contours[largestContourIndex];

	VIZ_STEP
	{
		VIZ_MAT = Mat::zeros(image.size(), CV_8U);

		cv::drawContours(VIZ_MAT, contours, largestContourIndex, 255, -1);
	}

	//
	// Find bounding square
	//

	cv::RotatedRect boundingRect = cv::minAreaRect(chosenContour);
	auto size = boundingRect.size;
	boundingRect.size.width = boundingRect.size.height = (size.width + size.height) / 2;
	square.resize(4);
	boundingRect.points(&square[0]);

	printTimeSinceLastCall("Find Bounding square");

	//
	// Reorder square points into CW order
	//

	Point2f centroid = (square[0] + square[1] + square[2] + square[3]) / 4;

	std::sort(square.begin(),
			square.end(),
			[centroid](const Point2f &p1, const Point2f &p2) {
			Point2f v1 = p1 - centroid;
			Point2f v2 = p2 - centroid;
			return std::atan2(-v1.y, v1.x) > std::atan2(-v2.y, v2.x);
			});

	printTimeSinceLastCall("Reorder points");

	//
	// Find the missing edge.
	// The missing edge is the edge whose midpoint
	// is farther from any contour point than the midpint
	// of any other edge.
	//

	float maxMinDistance = -1;
	int maxMinDistanceIndex = -1;

	for (unsigned int i = 0; i < square.size(); i++) {
		Point2f edgeCenter = (square[i] + square[(i + 1) % square.size()]) / 2;

		float currMinDistance = FLT_MAX;

		for (unsigned int k = 0; k < chosenContour.size(); k++) {
			float dx = edgeCenter.x - chosenContour[k].x;
			float dy = edgeCenter.y - chosenContour[k].y;

			float sqrDist = dx * dx + dy * dy;

			currMinDistance = std::min(currMinDistance, sqrDist);
		}

		if (currMinDistance > maxMinDistance) {
			maxMinDistance = currMinDistance;
			maxMinDistanceIndex = i;
		}
	}

	std::rotate(square.begin(), square.begin() + (maxMinDistanceIndex + 1) % square.size(), square.end());

	printTimeSinceLastCall("Find missing edge and finish findOpenSquare");

	return true;
}

void findRailMarkers(const Mat &image, vector<Point2f> &outMarkers)
{
	Mat binary;
	binarizeImageInv(image, binary);

	//
	// Find the contours
	//

	vector<vector<Point2i>> contours;
	vector<Vec4i> hierarchy;

	{
		Mat binaryCopy = binary.clone();

		cv::findContours(binaryCopy,
				contours,
				hierarchy,
				CV_RETR_TREE,
				CV_CHAIN_APPROX_NONE);
	}

	//
	// Select contour with largest area
	//

	int largestContour = -1;

	{
		double maxArea = 0;

		for (int i = 0; i < (int)contours.size(); i++) {
			double currArea = cv::contourArea(contours[i]);

			if (currArea > maxArea) {
				maxArea = currArea;
				largestContour = i;
			}
		}
	}

	//
	// Find all inner contours of the largest contour
	//

	vector<vector<Point2i>> markerContours;

	if (largestContour >= 0) {
		int nextMarker = hierarchy[largestContour][2];

		while (nextMarker >= 0) {
			markerContours.push_back(contours[nextMarker]);
			nextMarker = hierarchy[nextMarker][0];
		}
	}

	//
	// The markers are the centroids of those contours
	//

	outMarkers.resize(0);

	{
		for (auto &contour : markerContours) {
			Point2f centroid;
			Point2f sum{0, 0};

			for (auto point : contour) {
				sum += Point2f{point};
			}

			centroid = Point2f{sum.x / contour.size(), sum.y / contour.size()};
			outMarkers.push_back(centroid);
		}
	}

	VIZ_STEP
	{
		VIZ_MAT = image.clone();

		cv::drawContours(VIZ_MAT, contours, largestContour, Scalar{66, 66, 244}, 2);
		cv::drawContours(VIZ_MAT, markerContours, -1, Scalar{66, 244, 66}, 2);

		for (auto m : outMarkers) {
			cv::circle(VIZ_MAT, m, 3, Scalar{244, 66, 66}, -1);
		}
	}
}

void findFloorTileMarkers(const Mat &image, vector<Point2f> &outMarkers, int thresholdC = 8 + 5 - 3 - 3, int houghThreshold = 40 - 10)
{
	CV_Assert(image.type() == CV_8U || image.type() == CV_8UC3);

	Mat gray;
	if (image.type() != CV_8U) {
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}
	else {
		gray = image.clone();
	}

	cv::blur(gray, gray, Size{3, 3});

	Mat binary;

	cv::adaptiveThreshold(
			gray,
			binary,
			255,
			cv::ADAPTIVE_THRESH_MEAN_C,
			cv::THRESH_BINARY_INV,
			19 * 2 + 1,
			thresholdC);

	VIZ_STEP
	{
		VIZ_MAT = binary.clone();
	}

	vector<Vec4i> lines;
	cv::HoughLinesP(binary, lines, 1, CV_PI / 180, houghThreshold, 60, 10);

	vector<Point2f> intersections;

	for (size_t i = 0; i < lines.size(); i++) {
		for (size_t k = 0; k < lines.size(); k++) {
			//
			// Given a segment between points P and Q,
			// we represent it as a base point P and vector V = Q - P.
			// The segment consists of all points P + t * V where t is in [0, 1].
			//
			// We have two lines:
			// Pa + x * Va
			// Pb + y * Vb
			//
			// Their intersection gives us two equations:
			// Pa.x + x * Va.x = Pb.x + y * Vb.x
			// Pa.y + x * Va.y = Pb.y + y * Vb.y
			//
			// We transform this into the following matrix:
			// [ (Va.x) (-Vb.x) | (Pb.x - Pa.x) ]
			// [ (Va.y) (-Vb.y) | (Pb.y - Pa.y) ]
			//
			// We solve this matrix using Cramer's Rules, as described here:
			// https://www.cliffsnotes.com/study-guides/algebra/algebra-ii/linear-sentences-in-two-variables/ --
			// linear-equations-solutions-using-determinants-with-two-variables
			//
			// The determinant tells us if there is a solution, and we know it is valid if
			// x and y are both in range [0, 1] (on the segment).
			//
			// We use the dot product of Va and Vb to calculate the
			// cosine of the intersection angle of the segments.
			//
			// Finally, the intersection point is Pa + x * Va. (which is also Pb + y * Vb)
			//

			auto &la = lines[i];
			auto &lb = lines[k];

			Point2f pa{(float)la[0], (float)la[1]};
			Point2f pb{(float)lb[0], (float)lb[1]};

			Point2f va{(float)(la[2] - la[0]), (float)(la[3] - la[1])};
			Point2f vb{(float)(lb[2] - lb[0]), (float)(lb[3] - lb[1])};

			float determinant = va.x * (-vb.y) - va.y * (-vb.x);
			float determinantX = (pb.x - pa.x) * (-vb.y) - (pb.y - pa.y) * (-vb.x);
			float determinantY = va.x * (pb.y - pa.y) - va.y * (pb.x - pa.x);

			if (std::abs(determinant) <= 0.001)
				continue;

			float x = determinantX / determinant;
			float y = determinantY / determinant;

			float magnitudeA = std::sqrt(va.x * va.x + va.y * va.y);
			float magnitudeB = std::sqrt(vb.x * vb.x + vb.y * vb.y);

			const float padding = 30 * 2;

			float minX = -padding / magnitudeA;
			float maxX = 1 + padding / magnitudeA;

			float minY = -padding / magnitudeB;
			float maxY = 1 + padding / magnitudeB;

			if (x < minX || x > maxX || y < minY || y > maxY)
				continue;

			float dot = va.x * vb.x + va.y * vb.y;
			float cosAngle = dot / (magnitudeA * magnitudeB);

			if (std::abs(cosAngle) > 0.3)
				continue;

			Point2f intersection = pa + x * va;

			intersections.push_back(intersection);
		}
	}

	vector<bool> isMerged(intersections.size(), false);
	vector<Point2f> finalPoints;

	const int sqrMergeThreshold = 20 * 20;

	for (size_t i = 0; i < intersections.size(); i++) {
		if (isMerged[i])
			continue;

		Point2f sum = intersections[i];
		int count = 1;

		for (size_t k = i + 1; k < intersections.size(); k++) {
			if (isMerged[k])
				continue;

			Point2i diff = intersections[i] - intersections[k];
			int sqrDistance = diff.x * diff.x + diff.y * diff.y;

			if (sqrDistance > sqrMergeThreshold)
				continue;

			sum += intersections[k];
			count++;
			isMerged[k] = true;
		}

		finalPoints.push_back(sum / count);
	}

	outMarkers = finalPoints;

	VIZ_STEP
	{
		VIZ_MAT = image.clone();

		for (size_t i = 0; i < lines.size(); i++) {
			Vec4i l = lines[i];
			cv::line(VIZ_MAT, Point{l[0], l[1]}, Point{l[2], l[3]}, Scalar{0,0,255});
		}

		for (auto point : finalPoints) {
			cv::circle(VIZ_MAT, point, 5, Scalar{255, 0, 0}, -1);
		}
	}
}

Point2i orthoProject(Point3f point, Affine3f cameraMatrix)
{
	point = cameraMatrix * point;
	return Point2i{(int)point.x, (int)point.z};
}

float rotationYaw(const Mat &rotation)
{
	float zx = rotation.at<float>(0, 2);
	float zz = rotation.at<float>(2, 2);
	float atanY = zx;
	float atanX = zz;

	float yawCCW = std::atan2(atanY, atanX);

	return -yawCCW;
}

void drawViewOrthographicXZ(
		Mat &screenBuffer,
		Size2i size,
		const char* name,
		Affine3f droneTransform,
		Point3f droneTarget,
		Vec4f droneControlErrors,
		Affine3f transform,
		bool drawPlaneIcon = false)
{
	screenBuffer.create(size.height, size.width, CV_8UC3);
	screenBuffer.setTo(Scalar{51, 51, 51});

	Point3f xBegin{-100 * 1000, 0, 0};
	Point3f xEnd = -xBegin;
	Point3f yBegin{0, -100 * 1000, 0};
	Point3f yEnd = -yBegin;
	Point3f zBegin{0, 0, -100 * 1000};
	Point3f zEnd = -zBegin;

	Scalar axesColor{242, 158, 106};

	cv::line(screenBuffer, orthoProject(xBegin, transform), orthoProject(xEnd, transform), axesColor);
	cv::line(screenBuffer, orthoProject(yBegin, transform), orthoProject(yEnd, transform), axesColor);
	cv::line(screenBuffer, orthoProject(zBegin, transform), orthoProject(zEnd, transform), axesColor);

	Point2i planePosition = orthoProject(droneTransform.translation(), transform);
	cv::circle(screenBuffer, planePosition, 3, Scalar{255, 255, 255}, 2);


	cv::putText(screenBuffer, name, {size.width - 50, 27}, cv::FONT_HERSHEY_PLAIN, 1, axesColor);

	if (drawPlaneIcon) {
		Mat currRotation{droneTransform.rotation()};

		float planeRotation = rotationYaw(Mat{droneTransform.rotation()});

		// http://www.challengers101.com/images/CompRose1.jpg
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
		};

		vector<Point2i> planeIconHalf2 = planeIconHalf1;
		std::reverse(planeIconHalf2.begin(), planeIconHalf2.end());

		for (auto &p : planeIconHalf2) {
			p = Point2i{-p.x, p.y};
		}

		vector<Point2i> planeIcon = planeIconHalf1;
		planeIcon.insert(planeIcon.begin(), planeIconHalf2.begin(), planeIconHalf2.end());

		for (auto &p : planeIcon) {
			float cosT = std::cos(planeRotation);
			float sinT = std::sin(planeRotation);
			p = Point2i{(int)(p.x * cosT - p.y * sinT), (int)(p.x * sinT + p.y * cosT)};

			p /= 2;
			p += planePosition;
		}

		vector<vector<Point2i>> planeIconWrapper = {planeIcon};

		cv::polylines(screenBuffer, planeIconWrapper, false, Scalar{255, 255, 255});
	}

	Point2i droneTargetPosition = orthoProject(droneTarget, transform);
	int rectSide = 10;
	cv::rectangle(screenBuffer, Rect2i{droneTargetPosition.x - rectSide / 2, droneTargetPosition.y - rectSide / 2, rectSide, rectSide}, Scalar{66, 66, 244}, 2);

	Point3f controlErrors{droneControlErrors[0], droneControlErrors[1], -droneControlErrors[2]};
	//controlErrors2D = droneTransform * controlErrors;

	Point2i dronePosition = orthoProject(droneTransform.translation(), transform);
	Point2i shiftedControlErrors = orthoProject(droneTransform * controlErrors, transform);
	cv::arrowedLine(screenBuffer, dronePosition, shiftedControlErrors, Scalar{66, 66, 244}, 2);
}

void drawFlightViz(Mat &screenBuffer,
		Mat &cameraFeed,
		Affine3f droneTransform,
		float frameRate,
		Point3f droneTarget,
		Vec4f droneControlErrors,
		bool flightModeOn)
{
	Mat q1, q2, q3, q4;

	q1 = cameraFeed;

	Rect2i panelSize{0, 0, q1.cols, q1.rows};

	Mat rotationTop = (cv::Mat_<float>(3, 3) <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1);
	float scaleTop = 1;
	Vec3f shiftTop{(float)panelSize.width / 2, 0, (float)panelSize.height / 2};

	Mat rotationBack = (cv::Mat_<float>(3, 3) <<
		1, 0, 0,
		0, 0, 1,
		0, -1, 0);
	float scaleBack = 1;
	Vec3f shiftBack{(float)panelSize.width / 2, 0, (float)panelSize.height * 3 / 4};

	Mat rotationLeft = (cv::Mat_<float>(3, 3) <<
		0, 0, 1,
		1, 0, 0,
		0, -1, 0);
	float scaleLeft = 1;
	Vec3f shiftLeft{(float)panelSize.width / 2, 0, (float)panelSize.height * 3 / 4};

	drawViewOrthographicXZ(q2, panelSize.size(), "TOP" , droneTransform, droneTarget, droneControlErrors, Affine3f{rotationTop * scaleTop, shiftTop}, true);
	drawViewOrthographicXZ(q3, panelSize.size(), "LEFT", droneTransform, droneTarget, droneControlErrors, Affine3f{rotationLeft * scaleLeft, shiftLeft});
	drawViewOrthographicXZ(q4, panelSize.size(), "BACK", droneTransform, droneTarget, droneControlErrors, Affine3f{rotationBack * scaleBack, shiftBack});

	if (!flightModeOn) {
		auto text = "Flight Mode OFF. Press X to begin flight.";
		auto font = cv::FONT_HERSHEY_SIMPLEX;
		auto fontScale = 0.65;
		auto thickness = 2;

		int tmpBaseline;
		Size textSize = cv::getTextSize(text, font, fontScale, thickness, &tmpBaseline);

		cv::putText(q2,
				text, Point{(q2.cols - textSize.width) / 2, 20 + textSize.height},
				font, fontScale, Scalar{0, 0, 255}, thickness);
	}

	Size2i totalSize{panelSize.width * 2 + 1, panelSize.height * 2 + 1};

	screenBuffer.create(totalSize, CV_8UC3);
	screenBuffer.setTo(Scalar{255, 255, 255});

	q1.copyTo(Mat{screenBuffer, Rect2i{0, 0, panelSize.width, panelSize.height}});
	q2.copyTo(Mat{screenBuffer, Rect2i{panelSize.width + 1, 0, panelSize.width, panelSize.height}});
	q3.copyTo(Mat{screenBuffer, Rect2i{0, panelSize.height + 1, panelSize.width, panelSize.height}});
	q4.copyTo(Mat{screenBuffer, Rect2i{panelSize.width + 1, panelSize.height + 1, panelSize.width, panelSize.height}});
}

Vec4f calculateControlErrors(Vec3f currPos, Mat currRotation, Vec3f targetPos)
{
	float yawCW = rotationYaw(currRotation);

	Affine3f yawRotation{Vec3f{0, 1, 0} * -yawCW};

	Point3f droneWorldForward = yawRotation * Vec3f{0, 0, -1};
	Point3f droneWorldRight = yawRotation * Vec3f{1, 0, 0};
	Point3f droneWorldUp = yawRotation * Vec3f{0, 1, 0};

	Point3f target = targetPos - currPos;

	Vec4f errors {
		target.dot(droneWorldRight),
		target.dot(droneWorldUp),
		target.dot(droneWorldForward),
		0 - yawCW
	};

	return errors;
}

struct ChannelBounds
{
	int min;
	int zero;
	int max;
};

int lerp(float t, int a, int b)
{
	return (int)((1 - t) * a + t * b);
}

int transformIntoChannel(float normalizedValue, ChannelBounds bounds)
{
	float t = normalizedValue;
	if (t >= 0) {
		return lerp(t, bounds.zero, bounds.max);
	}
	else {
		return lerp(-t, bounds.zero, bounds.min);
	}
}

Vec4b controlsToDroneChannels(Vec4f normalizedInput)
{
	ChannelBounds rightBounds{127 - 30, 64, 0 + 30};
	ChannelBounds forwardBounds{0 + 30, 64, 127 - 30};
	ChannelBounds upBounds{10, 58, 127};
	ChannelBounds clockwiseBounds{127, 64, 0};

	int right = transformIntoChannel(normalizedInput[0], rightBounds);
	int up = transformIntoChannel(normalizedInput[1], upBounds);
	int forward = transformIntoChannel(normalizedInput[2], forwardBounds);
	int clockwise = transformIntoChannel(normalizedInput[3], clockwiseBounds);

	// NOTE(Andrey): IMPORTANT: We change the order of the values here!
	return {(uchar)right, (uchar)forward, (uchar)up, (uchar)clockwise};
}

enum Direction
{
	DIRECTION_UP = 1 << 0,
	DIRECTION_RIGHT = 1 << 1,
	DIRECTION_DOWN = 1 << 2,
	DIRECTION_LEFT = 1 << 3,
	DIRECTION_ALL = 0xF,
};

Direction pointDirection(Point2f p)
{
	float dotLeftUp = p.dot(Point2f{-1, -1});
	float dotRightUp = p.dot(Point2f{1, -1});

	if (dotLeftUp >= 0) {
		if (dotRightUp >= 0) {
			return DIRECTION_UP;
		}
		else {
			return DIRECTION_LEFT;
		}
	}
	else {
		if (dotRightUp >= 0) {
			return DIRECTION_RIGHT;
		}
		else {
			return DIRECTION_DOWN;
		}
	}
}

int closestPointTo(
		const vector<Point2f> &points,
		Point2f targetPoint,
		int directionFlags = DIRECTION_ALL,
		int exludePoint = -1,
		float minDistance = -1,
		float maxDistance = FLT_MAX)
{
	int closestPoint = -1;
	float highSquareLimit = maxDistance * maxDistance;
	float lowSquareLimit = (minDistance < 0) ? -1 : minDistance * minDistance;

	for (int i = 0; i < (int)points.size(); i++) {
		float dx = points[i].x - targetPoint.x;
		float dy = points[i].y - targetPoint.y;

		float sqrDist =  dx * dx + dy * dy;

		Direction direction = pointDirection(points[i] - targetPoint);

		if (sqrDist < highSquareLimit &&
				sqrDist >= lowSquareLimit &&
				i != exludePoint &&
				(direction & directionFlags)) {
			closestPoint = i;
			highSquareLimit = sqrDist;
		}
	}

	return closestPoint;
}

int main(int argc, char* argv[])
{
	//                                                                             #####
	// In Pattern mode, the drone stabilizes above a black pattern that looks like #   #
	//                                                                             #   #
	//
	// In Rail mode, the drone flies according to white shapes embedded in a
	// black line placed across the floor. It can be controlled with commands like
	// "go forward" and "go right". The rail looks like this: (# are white squares,
	// the rest is black)
	//
	// *--------------------------------------*
	// *                                      |
	// *   #     #     #     #     #     #    |
	// *                                      |
	// *--------------------------------------*
	//
	// In Floor mode, the drone flies according the corners of floor tiles.
	// It can be controlled with commands like "go forward" and "rotate right".
	// The tile edges have to be clearly visible for this to work well.
	//
	enum class TrackingMode { Pattern, Rail, Floor };
	TrackingMode activeTrackingMode = TrackingMode::Pattern;

	//
	// TX is the wireless camera we were using, and which required special handling:
	// In TX mode the image is flipped vertically, a small black area near
	// the borders is removed, and simple de-interlacing is applied.
	//
	bool txMode = false;

	//
	// DEVO mode is for the older remotes that use a physical Walkera DEVO
	// drone controller. Leave this off of the small new remotes.
	//
	bool devoMode = false;

	//
	// If we are running on a respberry computer,
	// we have the camera interface to grab photos from the RaspberryCam.
	//
	bool berryMode = false;

	//
	// Images on RaspberryCam are too big,
	// we need to scale them down.
	//
	float scaleFactor = 1.0;

	//
	// Visualizations are privilege.
	// Too heavy to run on the raspberry.
	//
	bool GUIOn = true;
	
	//
	// Sometimes we don't want to create this window.
	//
	bool configOn = true;
	
	QuadSerial serial;
	serial.openDefault();
	serial.printErrorIfNotConnected();

	vector<Pid> pidControllers = {
		{0.04f, 0, 0},
		{0.06f, 0, 0},
		{0.04f, 0, 0},
		{0.3f, 0, 0},
	};

	int p_xz = 4;
	int i_xz = 10;
	int d_xz = 28;

	int p_y = 4;
	int i_y = 10;
	int d_y = 30;

	int p_r = 30;
	int i_r = 0;
	int d_r = 0;

	int i_val_x = 50;
	int i_val_y = 99;
	int i_val_z = 50;
	int i_val_r = 50;

	int smoothing_factor_slider = 40;

	int cameraIndex = 0;

	for(int i = 1; i < argc; i++) {
		std::string temp = std::string(argv[i]);
		if (temp == "-ngui")
			GUIOn = !GUIOn;
		else if	(temp == "-nviz")
			VIZStepFunction = !VIZStepFunction;
		else if (temp == "-pi")
			berryMode = !berryMode;
		else if (temp == "-nconfig")
			configOn = !configOn;
		else if (temp.find_first_not_of(".0123456789") == std::string::npos) { // Returns true if it's a number
			if(std::stof(temp) == std::stoi(temp))
				cameraIndex = std::stoi(argv[i]);
			else
				scaleFactor = std::stof(argv[i]);
		}
	}

	if(configOn) {
		cv::namedWindow("config", cv::WINDOW_NORMAL);
		
		cv::createTrackbar("P xz", "config", &p_xz, 100);
		cv::createTrackbar("I xz", "config", &i_xz, 100);
		cv::createTrackbar("D xz", "config", &d_xz, 100);
		cv::createTrackbar("P y ", "config", &p_y, 100);
		cv::createTrackbar("I y ", "config", &i_y, 100);
		cv::createTrackbar("D y ", "config", &d_y, 100);
		cv::createTrackbar("P r ", "config", &p_r, 100);
		cv::createTrackbar("I r ", "config", &i_r, 100);
		cv::createTrackbar("D r ", "config", &d_r, 100);
		cv::createTrackbar("I val x ", "config", &i_val_x, 100);
		cv::createTrackbar("I val y ", "config", &i_val_y, 100);
		cv::createTrackbar("I val z ", "config", &i_val_z, 100);
		cv::createTrackbar("I val r ", "config", &i_val_r, 100);
		cv::createTrackbar("Smoothing factor", "config", &smoothing_factor_slider, 100);
	}

	raspicam::RaspiCam_Cv piCap;
	VideoCapture cap;

	if(berryMode) {
		std::cout << "Opening Camera..." << std::endl;
		piCap.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
		piCap.set( CV_CAP_PROP_FRAME_WIDTH, 320 );
		piCap.set( CV_CAP_PROP_FRAME_HEIGHT, 240 );
		
		if (!piCap.open())
		{
			std::cerr << "Error opening the camera" << std::endl;
			return -1;
		}
	}
	else {
		cap.open(cameraIndex);
		
		if (!cap.isOpened()) {
			std::cerr << "Error: Couldn't capture camera number " << cameraIndex << '\n';
			return -1;
		}
	
		cap.set(cv::CAP_PROP_FPS, 60);
	}

	Mat frame;

	Mat sonyEyeCameraMatrix = (cv::Mat_<float>(3, 3) <<
		5.38614e+02, 0., 3.10130e+02,
		0., 5.38112e+02, 2.27066e+02,
		0., 0., 1.);
	Mat txCameraMatrix = (cv::Mat_<float>(3, 3) <<
		6.7628576774457656e+02, 0., 3.0519865395809290e+02,
		0., 6.7561534030641053e+02, 2.4692172053127743e+02,
		0., 0., 1.);
	Mat PICameraMatrix = (cv::Mat_<float>(3, 3) <<
		297.8761532662123, 0., 158.6603882333692,
		0., 296.2831432511588, 120.2546307355579,
		0., 0., 1.);	
		
	Mat cameraMatrix = PICameraMatrix;
	if (berryMode) {
		cameraMatrix = PICameraMatrix;
	}

	// NOTE(Andrey): A down facing camera
	Affine3f droneCameraTransform{Vec3f{1, 0, 0} * (CV_PI / 2)};
	Affine3f invDroneCameraTransform = droneCameraTransform.inv();

	const int worldSquareSize = 21;
	vector<Point3f> worldSquare = {
		{0, 0, 0},
		{0, 0, (float)-worldSquareSize},
		{(float)worldSquareSize, 0, (float)-worldSquareSize},
		{(float)worldSquareSize, 0, 0}
	};
	vector<Point2f> cameraSquare;

	Affine3f droneTransform;
	Vec3f smoothPosition{0, 0, 0};
	float smoothingFactor = 0;

	Point3f droneTarget;

	if (activeTrackingMode == TrackingMode::Pattern) {
		droneTarget = {worldSquareSize, 160, -worldSquareSize};
		droneTarget = {worldSquareSize / 2, 145, -worldSquareSize / 2};
	}
	else {
		droneTarget = {worldSquareSize / 2, 230, -worldSquareSize / 2};
	}

	int64 lastFrameTickCount = 0;

	vector<Point2f> lastMarkers;
	vector<int> lastMarkersId;
	vector<int> lastMarkersAge;
	int nextMarkerId = 0;

	int baseMarker = -1;
	int targetMarker = -1;

	Direction nextStepDirecion = (Direction)(DIRECTION_LEFT | DIRECTION_UP | DIRECTION_RIGHT);

	Point2f lastBaseMarker;
	Point2f lastTargetMarker;

	bool markerTransition = false;
	float markerTransitionSpeed = 0.5 * 0.5;
	float markerTransitionT = 0;

	float sizeChangeSpeed = 0.3;
	float rotationChangeSpeed = 0.5 * 0.3;

	float squareStartSize = 1;
	float squareStartRotation = 0;

	bool stepForward = false;
	bool autoStep = false;

	bool landing = false;
	float landingProgress = 0;
	Vec4f neutralControls;

	bool takeoff = false;
	float takeoffProgress = 0;

	bool flightModeOn = false;
	bool paused = false;
	int pressedKey = 0;

	while (pressedKey != 'q') {
		printTimeSinceLastCall("************** Entered Loop *****************");
		if (!paused) {
			if(berryMode) {
				piCap.grab();
				piCap.retrieve(frame);
			}
			else {
				cap >> frame;
			}
			printTimeSinceLastCall("Photo grabing");
			if(scaleFactor != 1.0) {
				resize(frame, frame, Size(), scaleFactor, scaleFactor, cv::INTER_NEAREST);
			}
			printTimeSinceLastCall("Resizing");
			
			if (txMode) {

				//
				// Deinterlace and flip
				//

				// TODO: Do solve PnP in the original frame

				frame = frame(Range{5, frame.rows}, Range{0, frame.cols - 7});

				// NOTE(Andrey): From experiments we found the odd lines are
				// more recent in both TX05 and TX03.

				for (int i = 0; i < frame.rows - 1; i += 2) {
					frame.row(i + 1).copyTo(frame.row(i));
				}

				const int horizontalAndVertical = -1;
				cv::flip(frame, frame, horizontalAndVertical);
			}
			printTimeSinceLastCall("TX Mode");
		}

		VIZ_STEP
		{
			VIZ_MAT = frame.clone();
		}

		printTimeSinceLastCall("VIZ_MAT clone");

		float deltaTime = (float)(cv::getTickCount() - lastFrameTickCount) / cv::getTickFrequency();
		lastFrameTickCount = cv::getTickCount();

		printTimeSinceLastCall("Time since last frame tick count");

		bool found;

		if (activeTrackingMode == TrackingMode::Rail || activeTrackingMode == TrackingMode::Floor) {
			vector<Point2f> markers;
			vector<int> markersId;
			vector<int> markersAge;

			if (activeTrackingMode == TrackingMode::Rail) {
				findRailMarkers(frame, markers);
			}
			else if (activeTrackingMode == TrackingMode::Floor) {
				findFloorTileMarkers(frame, markers);
			}

			int markerCount = markers.size();
			int lastMarkerCount = lastMarkers.size();

			{
				//
				// We try to match the markers of each frame with the markers
				// of the previous frame (using the marker ID). In this matching
				// process we give priority to veteran markers, by making sure
				// that the previous markers are sorted according to how long
				// they have been tracked.
				//
				// To make the system more robust, we introduce the concept of marker age.
				// New markers need to be tracked for some frames (while age is negative)
				// before they are displayed. Tracked markers stay with age equal to zero.
				// The age value of lost markers begins to increase (from zero)
				// until either it is found again, or becomes too old and dies.
				//

				const float maxSquareMovementDistance = 20 * 20;
				const int startAge = -7;
				const int endAge = 7;

				vector<bool> isMatched(markers.size(), false);
				vector<Point2f> sortedMarkers;
				vector<int> sortedMarkersId;
				vector<int> sortedMarkersAge;

				Point2f averageMovement{0, 0};
				int count = 0;

				for (int i = 0; i < lastMarkerCount; i++) {
					float minSquareDistance = FLT_MAX;
					int minIndex = -1;

					for (int k = 0; k < markerCount; k++) {
						if (isMatched[k])
							continue;

						float dx = lastMarkers[i].x - markers[k].x;
						float dy = lastMarkers[i].y - markers[k].y;

						float squareDistance = dx * dx + dy * dy;

						if (squareDistance < minSquareDistance) {
							minSquareDistance = squareDistance;
							minIndex = k;
						}
					}

					if (minIndex >= 0 && minSquareDistance <= maxSquareMovementDistance) {
						isMatched[minIndex] = true;
						sortedMarkers.push_back(markers[minIndex]);
						sortedMarkersId.push_back(lastMarkersId[i]);
						sortedMarkersAge.push_back(std::min(lastMarkersAge[i] + 1, 0));

						averageMovement += markers[minIndex] - lastMarkers[i];
						count++;
					}
					else {
						if (0 <= lastMarkersAge[i] && lastMarkersAge[i] < endAge) {
							sortedMarkers.push_back(lastMarkers[i]);
							sortedMarkersId.push_back(lastMarkersId[i]);
							sortedMarkersAge.push_back(lastMarkersAge[i] + 1);
						}
					}
				}

				// TODO: Refactor

				if (count != 0) {
					averageMovement /= count;

					for (int i = 0; i < (int)sortedMarkersAge.size(); i++) {
						if (sortedMarkersAge[i] > 0) {
							sortedMarkers[i] += averageMovement;
						}
					}
				}

				for (int i = 0; i < markerCount; i++) {
					if (!isMatched[i]) {
						sortedMarkers.push_back(markers[i]);
						sortedMarkersId.push_back(nextMarkerId++);
						sortedMarkersAge.push_back(startAge);
					}
				}

				markers = sortedMarkers;
				markersId = sortedMarkersId;
				markersAge = sortedMarkersAge;

				// NOTE(Andrey): We may have added markers to the list
				markerCount = markers.size();

				lastMarkers = markers;
				lastMarkersId = markersId;
				lastMarkersAge = markersAge;
			}

			VIZ_STEP
			{
				VIZ_MAT = frame.clone();

				for (int i = 0; i < markerCount; i++) {
					cv::circle(VIZ_MAT, markers[i], 10, Scalar{0, 0, 255}, -1);
					cv::putText(VIZ_MAT, std::to_string(markersId[i]),
							markers[i] + Point2f{3, 3}, cv::FONT_HERSHEY_PLAIN, 1, Scalar{0});
				}
			}

			VIZ_STEP
			{
				VIZ_MAT = frame.clone();

				for (int i = 0; i < markerCount; i++) {
					if (markersAge[i] >= 0) {
						Scalar color = markersAge[i] == 0 ? Scalar{0, 0, 255} : Scalar{0};
						cv::circle(VIZ_MAT, markers[i], 10, color, -1);
						cv::putText(VIZ_MAT, std::to_string(markersId[i]),
								markers[i] + Point2f{10, 10}, cv::FONT_HERSHEY_PLAIN, 1, Scalar{0});
					}
				}
			}

			{
				vector<Point2f> adultMarkers;

				for (int i = 0; i < markerCount; i++) {
					if (markersAge[i] >= 0) {
						adultMarkers.push_back(markers[i]);
					}
				}

				markers = adultMarkers;
				markerCount = markers.size();
			}

			if (baseMarker < 0) {
				Point2f screenCenter{(float)(frame.cols / 2), (float)(frame.rows / 2)};
				baseMarker = closestPointTo(markers, screenCenter);
			}
			else {
				baseMarker = closestPointTo(markers, lastBaseMarker);
			}

			if (targetMarker < 0 && baseMarker >= 0) {
				targetMarker = closestPointTo(
						markers,
						markers[baseMarker],
						DIRECTION_UP,
						baseMarker);
			}
			else {
				targetMarker = closestPointTo(
						markers,
						lastTargetMarker,
						DIRECTION_ALL,
						baseMarker);
			}

			//
			// Stepping Animations
			//

			{
				auto approxEqual = [](float a, float b) -> bool {
					return std::abs(a - b) < 0.001;
				};

				auto moveTowards = [](float from, float to, float speed) -> float {
					float diff = to - from;
					if (diff > speed) {
						return from + speed;
					}
					else if (diff < -speed) {
						return from - speed;
					}
					else {
						return to;
					}
				};

				if (markerTransition && approxEqual(squareStartSize, 1) && approxEqual(squareStartRotation, 0)) {
					markerTransitionT += markerTransitionSpeed * deltaTime;

					if (markerTransitionT >= 1) {
						markerTransitionT = 1;
						markerTransition = false;
					}
				}

				squareStartSize = moveTowards(squareStartSize, 1, sizeChangeSpeed * deltaTime);
				squareStartRotation = moveTowards(squareStartRotation, 0, rotationChangeSpeed * deltaTime);
			}

			//
			// Stepping Forward
			//

			if (stepForward && !markerTransition && targetMarker >= 0) {
				int newTargetMarker = closestPointTo(
						markers,
						markers[targetMarker],
						nextStepDirecion,
						targetMarker);

				if (newTargetMarker >= 0) {
					auto distance = [](Point2f a, Point2f b) {
						float dx = a.x - b.x;
						float dy = a.y - b.y;

						return std::sqrt(dx * dx + dy * dy);
					};

					squareStartSize =
						distance(markers[baseMarker], markers[targetMarker]) /
						distance(markers[targetMarker], markers[newTargetMarker]);

					if (squareStartSize > 2) {
						squareStartSize = 2;
					}

					Point2f oldForward = markers[targetMarker] - markers[baseMarker];
					Point2f newForward = markers[newTargetMarker] - markers[targetMarker];

					squareStartRotation = std::atan2(oldForward.y, oldForward.x) - std::atan2(newForward.y, newForward.x);
					if (squareStartRotation > CV_PI) {
						squareStartRotation -= 2 * CV_PI;
					}
					else if (squareStartRotation < -CV_PI) {
						squareStartRotation += 2 * CV_PI;
					}

					baseMarker = targetMarker;
					targetMarker = newTargetMarker;
					markerTransitionT = 0;

					if (autoStep) {
						markerTransition = true;
					}
					else {
						stepForward = false;
					}
				}
			}

			if (baseMarker >= 0) {
				lastBaseMarker = markers[baseMarker];
			}
			if (targetMarker >= 0) {
				lastTargetMarker = markers[targetMarker];
			}

			if (baseMarker >= 0 && targetMarker >= 0) {
				Point2f forward = markers[targetMarker] - markers[baseMarker];
				Point2f right;

				cameraSquare.resize(4);
				cameraSquare[0] = markers[baseMarker] + forward * markerTransitionT;

				float cosT = std::cos(squareStartRotation);
				float sinT = std::sin(squareStartRotation);

				forward = Point2f{forward.x * cosT - forward.y * sinT, forward.x * sinT + forward.y * cosT};
				forward *= squareStartSize;
				right = Point2f{-forward.y, forward.x};

				cameraSquare[0] = cameraSquare[0] - forward / 2 - right / 2;
				cameraSquare[1] = cameraSquare[0] + forward;
				cameraSquare[2] = cameraSquare[1] + right;
				cameraSquare[3] = cameraSquare[0] + right;
			}

			found = baseMarker >= 0 && targetMarker >= 0;

			VIZ_STEP
			{
				VIZ_MAT = frame.clone();

				for (int i = 0; i < (int)markers.size(); i++) {
					cv::circle(VIZ_MAT, markers[i], 7, Scalar{0, 0, 255}, -1);
				}

				if (baseMarker >= 0) {
					cv::circle(VIZ_MAT, markers[baseMarker], 5, Scalar{0, 0, 0}, -1);
				}
				if (targetMarker >= 0) {
					cv::circle(VIZ_MAT, markers[targetMarker], 5, Scalar{66, 66, 244}, -1);
				}
				if (baseMarker >= 0 && targetMarker >= 0) {
					cv::arrowedLine(VIZ_MAT, markers[baseMarker], markers[targetMarker], Scalar{66, 66, 244}, 2);
					for (unsigned int i = 0; i < cameraSquare.size(); i++) {
						Scalar color = (i == cameraSquare.size() - 1) ? Scalar{66, 66, 244} : Scalar{66, 244, 66};
						cv::line(VIZ_MAT, cameraSquare[i], cameraSquare[(i + 1) % cameraSquare.size()], color, 3);
					}
				}
			}
		}
		else {
			printTimeSinceLastCall("Tracking Floor/Tiles section");
			found = findOpenSquare(frame, cameraSquare);

			VIZ_STEP
			{
				VIZ_MAT = frame.clone();

				for (unsigned int i = 0; i < cameraSquare.size(); i++) {
					Scalar color = (i == cameraSquare.size() - 1) ? Scalar{66, 66, 244} : Scalar{66, 244, 66};
					cv::line(VIZ_MAT, cameraSquare[i], cameraSquare[(i + 1) % cameraSquare.size()], color, 3);
				}
			}
		}
		printTimeSinceLastCall("Tracking OpenSquare section");

		if (!found) {
			std::cerr << "NOT FOUND" << std::endl;
		}

		if (found) {
			Mat rvec;
			Mat tvec;

			found = cv::solvePnP(worldSquare,
					cameraSquare,
					cameraMatrix,
					Mat{},
					rvec,
					tvec);

			printTimeSinceLastCall("SolvePnP");

			rvec.convertTo(rvec, CV_32F);
			tvec.convertTo(tvec, CV_32F);
			Affine3f cameraTransform = Affine3f{rvec, tvec};
			// The square doesn't move, we are moving.
			cameraTransform = cameraTransform.inv();

			// We found the camera, but we want to find the drone
			droneTransform = cameraTransform * invDroneCameraTransform;

			Vec3f pos = droneTransform.translation();
			smoothPosition = smoothingFactor * smoothPosition + (1 - smoothingFactor) * pos;

			droneTransform.translation(smoothPosition);
		}

		pidControllers[0].kp = p_xz / 1000.0;
		pidControllers[0].ki = i_xz / 10000.0;
		pidControllers[0].kd = d_xz / 10000.0;

		pidControllers[1].kp = p_y / 1000.0;
		pidControllers[1].ki = i_y / 10000.0;
		pidControllers[1].kd = d_y / 10000.0;

		pidControllers[2].kp = p_xz / 1000.0;
		pidControllers[2].ki = i_xz / 10000.0;
		pidControllers[2].kd = d_xz / 10000.0;

		pidControllers[3].kp = p_r / 100.0;
		pidControllers[3].ki = i_r / 1000.0;
		pidControllers[3].kd = d_r / 1000.0;

		i_val_x = (pidControllers[0].scaledErrorSum - pidControllers[0].minIntegral) /
			(pidControllers[0].maxIntegral - pidControllers[0].minIntegral) * 100;
		i_val_y = (pidControllers[1].scaledErrorSum - pidControllers[1].minIntegral) /
			(pidControllers[1].maxIntegral - pidControllers[1].minIntegral) * 100;
		i_val_z = (pidControllers[2].scaledErrorSum - pidControllers[2].minIntegral) /
			(pidControllers[2].maxIntegral - pidControllers[2].minIntegral) * 100;
		i_val_r = (pidControllers[3].scaledErrorSum - pidControllers[3].minIntegral) /
			(pidControllers[3].maxIntegral - pidControllers[3].minIntegral) * 100;


		if(configOn) {
			cv::setTrackbarPos("I val x ", "config", i_val_x);
			cv::setTrackbarPos("I val y ", "config", i_val_y);
			cv::setTrackbarPos("I val z ", "config", i_val_z);
			cv::setTrackbarPos("I val r ", "config", i_val_r);
		}

		smoothingFactor = smoothing_factor_slider / 100.0f;

		Vec4f controlErrors = calculateControlErrors(droneTransform.translation(), Mat{droneTransform.rotation()}, droneTarget);
		Vec4f pidControl;

		printTimeSinceLastCall("PID");

		if (takeoff) {
			if (takeoffProgress < 1e-6) {
				flightModeOn = true;
			}

			takeoffProgress += deltaTime;

			if (takeoffProgress < 0.7f) {
				pidControl[1] = 1;
			}
			else if (takeoffProgress < 2) {
				pidControl[1] = -0.5;
			}
			else {
				takeoff = false;
				takeoffProgress = 0;

				// Reset tracking
				baseMarker = -1;
				targetMarker = -1;
			}
		}
		else if (landing) {
			if (landingProgress < 1e-6) {
				// We want to get the integral term from the PIDs.
				// For this we have to neutralize the proportional and derivative terms.
				// Putting the drone at the origin neutralizes the proportional term, and
				// doing this two times in a row neutralizes the derivative term.
				for (int i = 0; i < 4; i++) {
					pidControllers[i].calculate(0, deltaTime);
					neutralControls[i] = pidControllers[i].calculate(0, deltaTime);
				}
			}

			landingProgress += deltaTime;

			if (landingProgress < 2.8) {
				pidControl = neutralControls;
				pidControl[1] = neutralControls[1] - 0.2f;
			}
			else {
				flightModeOn = false;
				landing = false;
				landingProgress = 0;
			}
		}
		else if (found && flightModeOn) {
			for (int i = 0; i < 4; i++) {
				pidControl[i] = pidControllers[i].calculate(controlErrors[i], deltaTime);
			}
		}
		else {
			for (int i = 0; i < 4; i++) {
				pidControl[i] = 0;
			}

			if (!flightModeOn) {
				pidControl[1] = -10;
			}
		}

		printTimeSinceLastCall("Take off and Landing");

		Vec4b channelControls = controlsToDroneChannels(pidControl);

		if (!flightModeOn) {
			channelControls = Vec4b{64, 64, 0, 64};
		}

		if (serial.isOpened()) {
			bool success;

			// TODO(Andrey): Work with floats instead
			if (devoMode) {
				success = serial.sendDevo(
						channelControls[0],
						channelControls[1],
						channelControls[2],
						channelControls[3]);
			}
			else{
				success = serial.send(
						((int)channelControls[0] - 64) * 2,
						((int)channelControls[1] - 64) * 2,
						((int)channelControls[2] - 64) * 2,
						((int)channelControls[3] - 64) * 2);
			}

			if (!success)
				std::cout << "Failed to send to Arduino" << std::endl;

			printTimeSinceLastCall("Send drone actions");
		}

		//printf("control errors: %6.1f, %6.1f, %6.1f, %6.1f cm\n",
		//		controlErrors[0],
		//		controlErrors[1],
		//		controlErrors[2],
		//		controlErrors[3]);

		printTimeSinceLastCall("Control errors to console");

		//
		// Draw GUI
		//
		{
			Mat displayedFrame = VIZ_MAT.clone();

			if (displayedFrame.empty()) {
				displayedFrame = Mat{32, 32, CV_8UC3};
			}
			else if (displayedFrame.type() == CV_8U) {
				cv::cvtColor(displayedFrame, displayedFrame, cv::COLOR_GRAY2BGR);
			}

			if(GUIOn) {
				drawFlightViz(displayedFrame,
					displayedFrame,
					droneTransform,
					59.9,
					droneTarget,
					pidControl * 100,
					flightModeOn);
			}
			
			if(VIZStepFunction) {
				cv::imshow("w", displayedFrame);
			}
			showFrameRateInTitle("w");
		}
		printTimeSinceLastCall("Draw GUI");

		pressedKey = cv::waitKey(1);

		//std::cout << pressedKey << std::endl;

		printTimeSinceLastCall("cv::waitKey(1)");

		const int ARROW_LEFT = 65361;
		const int ARROW_UP = 65362;
		const int ARROW_RIGHT = 65363;
		const int ARROW_DOWN = 65364;
		const int BACK_BUTTON = 269025062;
		const int HOME_BUTTON = 269025048;
		const int ENTER = 13;
		const int PI_ENTER = 141;
		const int PI_BACK_BUTTON = 38;

		switch (pressedKey) {
			case ' ':
				paused = !paused;
				break;
			case ENTER:
			case PI_ENTER:
			case 'x':
				flightModeOn = !flightModeOn;
				break;
			case 'p':
				landing = true;
				break;
			case 't':
				takeoff = true;
				break;
			case BACK_BUTTON:
			case PI_BACK_BUTTON:
			case 'i':
				// NOTE: Reset integral
				for (auto &pid : pidControllers)
					pid.scaledErrorSum = 0;
				break;
			case 'n':
				if (baseMarker >= 0 && targetMarker >= 0) {
					baseMarker = targetMarker;
					lastBaseMarker = lastTargetMarker;
					targetMarker = -1;
				}
				break;
			case HOME_BUTTON:
			case 'r':
				baseMarker = -1;
				targetMarker = -1;
				markerTransition = false;
				markerTransitionT = 0;
				squareStartSize = 1;
				squareStartRotation = 0;
				stepForward = false;
				break;
			case 'm':
				nextStepDirecion = (Direction)(DIRECTION_LEFT | DIRECTION_UP | DIRECTION_RIGHT);
				stepForward = true;
				markerTransition = true;
				break;
			case ARROW_LEFT:
			case 'h':
				nextStepDirecion = DIRECTION_LEFT;
				stepForward = true;
				markerTransition = true;
				break;
			case ARROW_UP:
			case 'k':
				nextStepDirecion = DIRECTION_UP;
				stepForward = true;
				markerTransition = true;
				break;
			case ARROW_RIGHT:
			case 'l':
				nextStepDirecion = DIRECTION_RIGHT;
				stepForward = true;
				markerTransition = true;
				break;
			case ARROW_DOWN:
			case 'j':
				nextStepDirecion = DIRECTION_DOWN;
				stepForward = true;
				markerTransition = true;
				break;
			case 'a':
				if (!autoStep) {
					stepForward = true;
					markerTransition = true;
				}
				autoStep = !autoStep;
				break;
		}

		printTimeSinceLastCall("User Commands");

		//
		// Prepare StepViz for next cycle
		//
		{
			stepViz.currentStep = -1;
			stepViz.vizMat.setTo(0xCC);

			if (pressedKey >= '0' && pressedKey <= '9') {
				if (pressedKey == '0') {
					stepViz.displayedStep = 9;
				}
				else {
					stepViz.displayedStep = pressedKey - '1';
				}
			}
			else if (pressedKey == '-') {
				if (stepViz.displayedStep > 0) {
					stepViz.displayedStep--;
				}
			}
			else if (pressedKey == '=') {
				stepViz.displayedStep++;
			}
		}
	}

	/*
	cv::viz::Viz3d window{"Flight Control"};

	window.showWidget("axes", cv::viz::WCoordinateSystem{20});
	window.showWidget("open_square", cv::viz::WPolyLine{worldSquare, cv::viz::Color{244, 66, 197}});
	window.setRenderingProperty("open_square", cv::viz::LINE_WIDTH, 3);
	window.showWidget("drone", cv::viz::WCube{Point3f{-10, -2, -10}, Point3f{10, 2, 10}, true});
	window.showWidget("drone_direction", cv::viz::WArrow{Point3f{0, 0, 0}, Point3f{20, 20, 20}});

	Affine3f camTransform{Vec3f{1, 0, 0} * (M_PI / 2)};

	while (!window.wasStopped()) {
		cap >> frame;
		Mat newFrame;

		for(int i = 4; i < frame.size().height - 1; i = i + 2)
		{
			newFrame.push_back(frame.row(i).colRange(0 , frame.size().width - 7));
			newFrame.push_back(frame.row(i).colRange(0 , frame.size().width - 7));
		}

		frame = newFrame;
		const int horizontalAndVertical = -1;
		cv::flip(frame, frame, horizontalAndVertical);

		bool found = findOpenSquare(frame, cameraSquare);

		if (found) {
			Mat rvec;
			Mat tvec;

			found = cv::solvePnP(worldSquare,
					cameraSquare,
					cameraMatrix,
					Mat{},
					rvec,
					tvec);


			Affine3f transform = Affine3f{Affine3d{rvec, tvec}};
			// The square doesn't move, we are moving.
			transform = transform.inv();
			transform = transform * camTransform.inv();

			window.setWidgetPose("drone", transform);
			window.setWidgetPose("drone_direction", transform);
		}

		if (found) {
			// NOTE(Andrey): Default Viz background
			window.setBackgroundColor(cv::viz::Color{2, 1, 1}, cv::viz::Color{240, 120, 120});
		}
		else {
			window.setBackgroundColor(cv::viz::Color{35, 35, 255}, cv::viz::Color{0, 0, 255});
		}

		window.spinOnce();
	}
	*/

	return 0;
}

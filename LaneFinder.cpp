#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <grabber/opencv_tools.hpp>

using namespace std;
using namespace cv;


class LaneFinder
{
private:
	// trackbar values
	int huemin = 73, satmin = 0, valmin = 96;
	int huemax = 140, satmax = 175, valmax = 255;
	int cannyLow = 50, cannyHigh = 200;
	int contourSize = 50;

	// Line contours finding globals
	vector<Point> leftLine;
	vector<Point> rightLine;
	vector<Point> leftLineOld;
	vector<Point> rightLineOld;
	Point2f leftLineBottom = { 122, 340 };
	Point2f rightLineBottom = { 178, 340 };

	// img processing
	Mat imgOrig, imgCut, imgGray, imgBlur, imgCanny;
	Mat imgHSV, imgColorThresh, imgCannyAndColor, imgTransf, imgLines, imgLinesInv;
	Mat M, Minv; // matrix for perspective transform

	double midPoint = 0;
	double midPointOld = 0;

public:
	LaneFinder() {
		trackbars();
	}
	double detectLanes(Mat img)
	{
		imgOrig = img;
		crop();
		findLines();
		transform2BEV();
		findLineContours();
		contours2Points();
		transform2img();
		display();
		return midPoint;
	}
	void display()
	{
		/*
		imshow("color", imgColorThresh);
		imshow("canny", imgCanny);
		imshow("canny+color", imgCannyAndColor);

		imshow("detected Lines", imgLines);
		imshow("invTr", imgLinesInv);
		*/
		imshow("color", imgColorThresh);
		imshow("canny", imgCanny);
		imshow("canny+color", imgCannyAndColor);
		imshow("BEV", imgTransf);
		imshow("detected Lines", imgLines);
		imshow("orig + Lines", imgOrig);
		waitKey(1);
	}
protected:

	void crop() // select the region of the image where lanes are
	{
		Rect roi(0, 260, 720, 100);  // x,y, width, height
		imgCut = imgOrig(roi);
	}

	// color detection
	void findLines() {

		// EDGE DETECTION: grayscale + blur + canny
		cvtColor(imgCut, imgGray, COLOR_BGR2GRAY);
		GaussianBlur(imgGray, imgBlur, Size(3, 3), 3, 3);

		Canny(imgBlur, imgCanny, cannyLow, cannyHigh);
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
		dilate(imgCanny, imgCanny, kernel);
		// COLOR DETECTION: HSV + Thresholding
		cvtColor(imgCut, imgHSV, COLOR_BGR2HSV);

		Scalar lower(huemin, satmin, valmin);
		Scalar upper(huemax, satmax, valmax);

		inRange(imgHSV, lower, upper, imgColorThresh);
		imgColorThresh.convertTo(imgColorThresh, CV_8UC1);
		dilate(imgColorThresh, imgColorThresh, kernel);
		imgCannyAndColor = imgCanny & imgColorThresh;

	}

	void transform2BEV()
	{
		// perspective transform
		vector<Point2f> input_points(4);
		input_points[0] = Point2f(261, 13);  // TL 
		input_points[1] = Point2f(423, 14);  // TR
		input_points[2] = Point2f(609, 102);  // BR
		input_points[3] = Point2f(147, 99);  // BL

		vector<Point2f> target_points(4); // every pixel is 10cm
		target_points[0] = Point2f(150 - 50, 400 - 302); // TL 
		target_points[1] = Point2f(150 + 30, 400 - 277);
		target_points[2] = Point2f(150 + 24, 400 - 55);
		target_points[3] = Point2f(150 - 20, 400 - 57);
		M = getPerspectiveTransform(input_points, target_points);
		Minv = getPerspectiveTransform(target_points, input_points);
		Size warpedSize = Size(300, 400);

		warpPerspective(imgCannyAndColor, imgTransf, M, warpedSize);
	}

	Mat PolynomialFit(vector<Point>& points, int order) {
		// https://stackoverflow.com/questions/68287740/polynomial-curve-fitting-in-opencv-c?rq=1
		// required for curve fitting into lanes - used in findLane
		Mat U(points.size(), (order + 1), CV_64F);
		Mat Y(points.size(), 1, CV_64F);

		for (int i = 0; i < U.rows; i++) {
			for (int j = 0; j < U.cols; j++) {
				U.at<double>(i, j) = pow(points[i].y, j);
			}
		}

		for (int i = 0; i < Y.rows; i++) {
			Y.at<double>(i, 0) = points[i].x;
		}

		cv::Mat K((order + 1), 1, CV_64F);
		if (U.data != NULL) {
			K = (U.t() * U).inv() * U.t() * Y;
		}
		return K;
	}

	Point2f contour2bottomPoint(vector<Point> contour)
	{
		// helper function to find the bottom point of a contour
		double mostBottom = 0;
		int mostBottomIndex;
		for (int i = 0; i < contour.size(); i++)
		{
			if (contour[i].y > mostBottom)
			{
				mostBottom = contour[i].y;
				mostBottomIndex = i;
			}
		}
		Point2f bottomPoint = { (float)contour[mostBottomIndex].x, (float)contour[mostBottomIndex].y };
		return bottomPoint;
	}

	int findLineContours()
	{
		vector<vector<Point>> contours; // list to store the different contours
		vector<Vec4i> hierarchy;

		// find all contours in the transformed image
		findContours(imgTransf, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		cvtColor(imgTransf, imgTransf, COLOR_GRAY2BGR);

		// drawContours(img, contours, -1, Scalar(255, 0, 255), 2);  //-1: draw all contours
		vector<vector<Point>> conPoly(contours.size()); // like the contour, but just stores the corner points


		float leftLineDist = -1000;
		float rightLineDist = -1000;

		// draw contours into black image
		imgLines = imgTransf.clone();
		rectangle(imgLines, Point(0, 0), Point(300, 400), Scalar(0, 0, 0), FILLED);

		if (contours.empty()) // no contours available
		{
			cout << "No line detected! Using old Line Lines." << endl;
			leftLine = leftLineOld;
			rightLine = rightLineOld;
			return -1;
		}

		// loop through all contours and find the ones closest to the points "left" and "right"
		for (int i = 0; i < contours.size(); i++)
		{
			int area = contourArea(contours[i]);

			if (area > contourSize) // skip too small contours
			{
				float distLeft = pointPolygonTest(contours[i], leftLineBottom, true); // returns 0 if in edge, 1 if inside and neg. values if distance
				if (distLeft > leftLineDist) // countour fits better than contour before
				{
					leftLineDist = distLeft;
					leftLine = contours[i];
				}
				float distRight = pointPolygonTest(contours[i], rightLineBottom, true);
				if (distRight > rightLineDist)
				{
					rightLineDist = distRight;
					rightLine = contours[i];
				}
			}
		}

		if (rightLine == leftLine) // rightLine and leftLine are equal -> just a single line was detected -> one old line must be used 
		{
			// loop over single Line and find the bottom point 
			cout << "Just a single line detected. Using old line" << endl;

			Point2f singleLineBottom = contour2bottomPoint(leftLine);
			float distRight = pointPolygonTest(rightLineOld, singleLineBottom, true);
			float distLeft = pointPolygonTest(leftLineOld, singleLineBottom, true);

			if (distRight > distLeft)
			{
				// single contour is the left line
				rightLine = rightLineOld;
				// left line can stay
			}
			else
			{
				// single contour is the right line
				leftLine = leftLineOld;
				// left line can stay
			}
		}

		// save current contours as old ones
		leftLineOld = leftLine;
		rightLineOld = rightLine;
		leftLineBottom = contour2bottomPoint(leftLine);
		rightLineBottom = contour2bottomPoint(rightLine);

		if ((rightLineBottom.x - leftLineBottom.x) < 20) // left and right line too close - lines not detected correctly - don´t use for tracking
		{
			// reintialize tracking
			leftLineBottom = { 122, 340 };
			rightLineBottom = { 178, 340 };
		}
	}

	void contours2Points()
	{
		// find quadratic coefficients
		Mat coeffsLeft = PolynomialFit(leftLine, 2);  // order 2 polynomial
		Mat coeffsRight = PolynomialFit(rightLine, 2);

		double middlePointy; // used to return middlePoint y coordinate

		// draw contours in red and blue
		drawContours(imgLines, vector<vector<Point>>(1, rightLine), -1, Scalar(255, 0, 0), FILLED);
		drawContours(imgLines, vector<vector<Point>>(1, leftLine), -1, Scalar(0, 0, 255), FILLED);

		// draw green points as indicator for polynomial regression
		for (int x_func = 360; x_func > 0; x_func -= 20)
		{
			// draw points on the Line, starting from bottom of the image
			double y_func_left = coeffsLeft.at<double>(0) + x_func * coeffsLeft.at<double>(1) + x_func * x_func *  coeffsLeft.at<double>(2);
			double y_func_right = coeffsRight.at<double>(0) + x_func * coeffsRight.at<double>(1) + x_func * x_func * coeffsRight.at<double>(2);
			circle(imgLines, Point(y_func_left, x_func), 3, Scalar(0, 255, 0), FILLED);
			circle(imgLines, Point(y_func_right, x_func), 3, Scalar(0, 255, 0), FILLED);
			if (x_func == 240) // 240 is approx. 8m in front of the ego vehicle
			{
				middlePointy = (y_func_right + y_func_left) / 2;
				circle(imgLines, Point(middlePointy, 240), 5, Scalar(255, 255, 255), FILLED);
			}

		}
		midPoint = middlePointy - 150;
		midPointOld = midPoint;

	}

	void transform2img()
	{
		// inverse transformation
		warpPerspective(imgLines, imgLinesInv, Minv, Size(720, 100));

		// undo the crop and fill border with black pixels
		copyMakeBorder(imgLinesInv, imgLinesInv, 260, 512 - 360, 0, 0, 0, Scalar(0, 0, 0));

		cvtColor(imgOrig, imgOrig, COLOR_BGRA2BGR);

		addWeighted(imgOrig, 1, imgLinesInv, 1, 0.0, imgOrig);
		putText(imgOrig, "Steering Target: " + to_string(midPoint), Point(20, 20), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 0, 0), 1);
	}

	void trackbars()
	{
		namedWindow("LaneDetector Trackbar", (800, 400)); // Trackbar window
		createTrackbar("Canny low", "LaneDetector Trackbar", &cannyLow, 255);
		createTrackbar("Canny high", "LaneDetector Trackbar", &cannyHigh, 255);
		createTrackbar("HueMin", "LaneDetector Trackbar", &huemin, 255); // value range 0 .. 255
		createTrackbar("HueMax", "LaneDetector Trackbar", &huemax, 255);
		createTrackbar("SatMin", "LaneDetector Trackbar", &satmin, 255);
		createTrackbar("SatMax", "LaneDetector Trackbar", &satmax, 255);
		createTrackbar("ValMin", "LaneDetector Trackbar", &valmin, 255);
		createTrackbar("ValMax", "LaneDetector Trackbar", &valmax, 255);
		createTrackbar("min contour size", "LaneDetector Trackbar", &contourSize, 2000);

	}

};

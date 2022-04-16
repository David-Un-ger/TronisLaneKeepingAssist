#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <communication/multi_socket.h>
#include <models/tronis/ImageFrame.h>
#include <grabber/opencv_tools.hpp>
#include <models/tronis/BoxData.h>

using namespace std;
using namespace cv;
class AccelerationController
{
	/*
	class that handles the speed and distance control of the vehicle.
	*/
private:
	int enableDistanceControl;
	double accelerationCmd = 0;  // command sent to Tronis

	// velocity control
	int setSpeed = 50;
	int P_speed = 80, D_speed = 200;
	double speedErrorOld;

	// distance control
	double vehicleDistance = 100;
	double vehicleDistanceOld = 100;
	double distanceErrorOld = 0;
	double distanceErrorSum = 0;
	int distanceWatchdog = 0;  // resets the distance if vehile disappears and dstance stays constant
	const int distanceWatchdogMax = 5;
	int P_distance = 30, I_distance = 10, D_distance = 30;
	int targetDistance = 20;

public:
	AccelerationController(bool enableDistanceCtrl) {
		enableDistanceControl = enableDistanceCtrl;
		accelerationControlTrackbar();
	}

	void setDistance(double distance) {
		vehicleDistance = distance;
	}

	double getCommand() {
		return accelerationCmd;
	}

	void control(double currentSpeed) {
		/*
		calls the distance or the speed controller and updates the accerationCmd
		*/
		currentSpeed = currentSpeed / 100000 * 3600; // cm/s -> km/h 
		if (enableDistanceControl)
			controlDistance(currentSpeed);
		if (!enableDistanceControl)
			controlSpeed(currentSpeed);
	}

	void controlSpeed(double currentSpeed) {
		double speedError = (double)setSpeed - currentSpeed;  // positive error if we are too slow.
		double speedErrorDifference = (speedError - speedErrorOld); // usually should be divided by time
		speedErrorOld = speedError;

		accelerationCmd = (double)P_speed / 100 * speedError + (double)D_speed / 100 * speedErrorDifference;
		if (accelerationCmd < 0)
			accelerationCmd = 0;
		cout << "Current Speed: " << to_string(currentSpeed) << " cmd: " << to_string(accelerationCmd) << endl;
	}

	void controlDistance(double currentSpeed) {
		if (vehicleDistance > 90) {  // normal velocity control
			controlSpeed(currentSpeed);
		}
		else  // vehicle is close - activate distance control
		{
			double distanceError = vehicleDistance - (double)targetDistance;  // positive if we are too far away.
			double distanceErrorDifference = (distanceError - distanceErrorOld); // usually should be divided by time
			distanceErrorOld = distanceError;
			distanceErrorSum += distanceError;
			accelerationCmd = (double)P_distance / 100 * distanceError + (double)I_distance / 100000 * distanceErrorSum + (double)D_distance / 10 * distanceErrorDifference;
			if (currentSpeed > setSpeed)  // don´t allow the car to go faster than the set speed
				accelerationCmd = 0;	
			if (currentSpeed < 2 && accelerationCmd < 0)  // don´t allow the car to drive backward
				accelerationCmd = 0;
			if (vehicleDistance == vehicleDistanceOld)  // check if vehicle is still present:
			{
				// distance didn´t change - no new detection since last control step
				distanceWatchdog++;
				if (distanceWatchdog >= distanceWatchdogMax)
				{
					distanceWatchdog = 0;
					vehicleDistance = 100;
					cout << "No vehicle detected - going back to normal control" << endl;
				}
			}
		}
		cout << "Current speed: " << to_string(currentSpeed) << " Vehicle distance: " << to_string(vehicleDistance) << " Cmd: " << to_string(accelerationCmd) << endl;
	}

	void accelerationControlTrackbar()
	{
		namedWindow("Acceleration Control", (600, 600)); // Trackbar window to select the control parameters
		createTrackbar("enableDistanceControl", "Acceleration Control", &enableDistanceControl, 1);
		createTrackbar("setSpeed", "Acceleration Control", &setSpeed, 100);
		createTrackbar("P_speed", "Acceleration Control", &P_speed, 1000);
		createTrackbar("D_speed", "Acceleration Control", &D_speed, 1000);
		createTrackbar("targetDistance", "Acceleration Control", &targetDistance, 30);
		createTrackbar("P_distance", "Acceleration Control", &P_distance, 1000);
		createTrackbar("I_distance", "Acceleration Control", &I_distance, 1000);
		createTrackbar("D_distance", "Acceleration Control", &D_distance, 1000);
	}
};

class SteeringController
{
private:
	// steering control
	double steeringCmd = 0;
	double midPoint = 0;
	double steeringErrorOld = 0;
	int P_steering = 80, D_steering = 80;
	int manualCommand = 0;

public:
	SteeringController() {
		steeringControlTrackbar();
	}
	double getCommand()
	{
		return steeringCmd;
	}
	void control(double midPoint)
	{
		// mid point positive: too far right -> steer left, negative: too far left -> steer right
		double steeringError = midPoint - 2;
		double steeringErrorDifference = steeringError - steeringErrorOld;
		steeringErrorOld = steeringError;

		steeringCmd = double(P_steering) / 10000 * midPoint + double(D_steering) / 10000 * steeringErrorDifference;

		// manualCommand 0: controller, 1: left, 2: straight, 3: right
		if (manualCommand > 0)
			steeringCmd = manualCommand - 2; // -> left: -1, straight: 0, right: 1
	}

	void steeringControlTrackbar()
	{
		namedWindow("Steering Control", (600, 400)); // Trackbar window
		createTrackbar("Auto | L | Straight | R", "Steering Control", &manualCommand, 3);
		createTrackbar("P", "Steering Control", &P_steering, 1000);
		createTrackbar("D", "Steering Control", &P_steering, 1000);
	}
};

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

class LaneAssistant
{
	// insert your custom functions and algorithms here
public:
	LaneAssistant()
	{
	}

	bool processData(tronis::CircularMultiQueuedSocket& socket)
	{
		string command = to_string(accController.getCommand()) + ";" + to_string(steerController.getCommand());
		socket.send(tronis::SocketData(command));
		return true;
	}

protected:
	std::string image_name_;
	cv::Mat image_;
	tronis::LocationSub ego_location_;
	tronis::OrientationSub ego_orientation_;
	double ego_velocity_;

	LaneFinder laneFinder;
	AccelerationController accController = AccelerationController(true);
	SteeringController steerController;

	// Function to detect lanes based on camera image
	// Insert your algorithm here
	void detectLanes()
	{
		double steeringControlTarget = laneFinder.detectLanes(image_);
		accController.control(ego_velocity_);
		steerController.control(steeringControlTarget);

		// do stuff
	}

	bool processPoseVelocity(tronis::PoseVelocitySub* msg)
	{
		ego_location_ = msg->Location;
		ego_orientation_ = msg->Orientation;
		ego_velocity_ = msg->Velocity;
		return true;
	}

	bool processObject(tronis::ModelDataWrapper data_model)
	{
		/*
		process the data from the bounding box sensor and update the acceleration controller with it
		*/
		//cout << data_model->ToString() << endl;
		tronis::BoxDataSub* sensorData = data_model.get_typed<tronis::BoxDataSub>();
		double vehicleDistance;
		for (int i = 0; i < sensorData->Objects.size(); i++)
		{
			tronis::ObjectSub& object = sensorData->Objects[i];
			if (object.BB.Extends.X > 100 && object.BB.Extends.X < 800 && object.BB.Extends.Y > 100 && object.BB.Extends.Y < 800)
			{
				vehicleDistance = sqrt(pow(object.Pose.Location.X / 100, 2) + pow(object.Pose.Location.Y / 100, 2)); // pythagoras
				if (vehicleDistance > 1)
				{
					//cout << "Detected " << object.ActorName.Value() << " in distance: " << to_string(vehicleDistance) << " m." << endl;
					accController.setDistance(vehicleDistance - 5.50); // the bbox sensr has 5.5m error / sits not in the front of the car
					return true;
				}
			}
		}
		
	}

	// Helper functions, no changes needed
public:
	// Function to process received tronis data
	bool getData(tronis::ModelDataWrapper data_model)
	{
		if (data_model->GetModelType() == tronis::ModelType::Tronis)
		{
			/*
			std::cout << "Id: " << data_model->GetTypeId()
				<< ", Name: " << data_model->GetName()
				<< ", Time: " << data_model->GetTime() << std::endl;
			*/
			// if data is sensor output, process data
			switch (static_cast<tronis::TronisDataType>(data_model->GetDataTypeId()))
			{
			case tronis::TronisDataType::Image:
			{
				processImage(
					data_model->GetName(),
					data_model.get_typed<tronis::ImageSub>()->Image);
				break;
			}
			case tronis::TronisDataType::ImageFrame:
			{
				const tronis::ImageFrame& frames(
					data_model.get_typed<tronis::ImageFrameSub>()->Images);
				for (size_t i = 0; i != frames.numImages(); ++i)
				{
					std::ostringstream os;
					os << data_model->GetName() << "_" << i + 1;

					processImage(os.str(), frames.image(i));
				}
				break;
			}
			case tronis::TronisDataType::ImageFramePose:
			{
				const tronis::ImageFrame& frames(
					data_model.get_typed<tronis::ImageFramePoseSub>()->Images);
				for (size_t i = 0; i != frames.numImages(); ++i)
				{
					std::ostringstream os;
					os << data_model->GetName() << "_" << i + 1;

					processImage(os.str(), frames.image(i));
				}
				break;
			}
			case tronis::TronisDataType::PoseVelocity:
			{
				processPoseVelocity(data_model.get_typed<tronis::PoseVelocitySub>());
				break;
			}
			case tronis::TronisDataType::Object:
			{
				processObject(data_model);
				break;
			}
			default:
			{
				//std::cout << data_model->ToString() << std::endl;
				processObject(data_model);
				break;
			}
			}
			return true;
		}
		else
		{
			//std::cout << data_model->ToString() << std::endl;
			return false;
		}
	}

protected:
	// Function to show an openCV image in a separate window
	void showImage(std::string image_name, cv::Mat image)
	{
		cv::Mat out = image;
		if (image.type() == CV_32F || image.type() == CV_64F)
		{
			cv::normalize(image, out, 0.0, 1.0, cv::NORM_MINMAX, image.type());
		}
		//cv::namedWindow(image_name.c_str(), cv::WINDOW_NORMAL);
		//cv::imshow(image_name.c_str(), out);
	}

	// Function to convert tronis image to openCV image
	bool processImage(const std::string& base_name, const tronis::Image& image)
	{
		//std::cout << "processImage" << std::endl;
		if (image.empty())
		{
			std::cout << "empty image" << std::endl;
			return false;
		}

		image_name_ = base_name;
		image_ = tronis::image2Mat(image);

		detectLanes();
		showImage(image_name_, image_);

		return true;
	}
};

// main loop opens socket and listens for incoming data
int main(int argc, char** argv)
{
	std::cout << "Welcome to lane assistant" << std::endl;

	// specify socket parameters
	std::string socket_type = "TcpSocket";
	std::string socket_ip = "127.0.0.1";
	std::string socket_port = "7778";

	std::ostringstream socket_params;
	socket_params << "{Socket:\"" << socket_type << "\", IpBind:\"" << socket_ip << "\", PortBind:" << socket_port << "}";

	int key_press = 0;	// close app on key press 'q'
	tronis::CircularMultiQueuedSocket msg_grabber;
	uint32_t timeout_ms = 500; // close grabber, if last received msg is older than this param

	LaneAssistant lane_assistant;

	while (key_press != 'q')
	{
		std::cout << "Wait for connection..." << std::endl;
		msg_grabber.open_str(socket_params.str());

		if (!msg_grabber.isOpen())
		{
			printf("Failed to open grabber, retry...!\n");
			continue;
		}

		std::cout << "Start grabbing" << std::endl;
		tronis::SocketData received_data;
		uint32_t time_ms = 0;

		while (key_press != 'q')
		{
			// wait for data, close after timeout_ms without new data
			if (msg_grabber.tryPop(received_data, true))
			{
				// data received! reset timer
				time_ms = 0;

				// convert socket data to tronis model data
				tronis::SocketDataStream data_stream(received_data);
				tronis::ModelDataWrapper data_model(
					tronis::Models::Create(data_stream, tronis::MessageFormat::raw));
				if (!data_model.is_valid())
				{
					std::cout << "received invalid data, continue..." << std::endl;
					continue;
				}
				// identify data type
				lane_assistant.getData(data_model);
				lane_assistant.processData(msg_grabber);
			}
			else
			{
				// no data received, update timer
				++time_ms;
				if (time_ms > timeout_ms)
				{
					std::cout << "Timeout, no data" << std::endl;
					msg_grabber.close();
					break;
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					key_press = cv::waitKey(1);
				}
			}
		}
		msg_grabber.close();
	}
	return 0;
}

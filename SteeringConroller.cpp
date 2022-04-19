#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


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


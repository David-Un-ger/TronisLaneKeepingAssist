#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


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
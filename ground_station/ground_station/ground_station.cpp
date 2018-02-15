#define _USE_MATH_DEFINES

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <stdio.h>
#include "Serial.h"
#include "Gimbal.h"
#include <Eigen/Dense>
#include <cmath>



using namespace std;
using namespace cv;

Point detectBlobs(Mat image);
void thresholdImage(Mat & image, Mat & imgThresholded);
int main1(int argc, char** argv);
int talkToGimbal(int argc, char** argv);
int main6(int argc, char** argv);

class ExtendedKalman
{
public:
	double m_dt; //The default timestep

	int m = 12; //The number of elements in our state vector
	int n = 12; //The number of elements in our sensor vector
	int k = 0; //The number of elements in our control vector

	Eigen::VectorXd x; //The state vector
	Eigen::VectorXd z; //The sensor vector
	Eigen::VectorXd u; //The contol vector

	Eigen::VectorXd calch(Eigen::VectorXd x, Eigen::VectorXd u); // the function expressing each sensor value in terms of the state variables
	Eigen::MatrixXd calcH(Eigen::VectorXd x, Eigen::VectorXd u); // the jacobian of the h vector wrt the state variables
	Eigen::VectorXd calcf(Eigen::VectorXd x, Eigen::VectorXd u, double dt); // the physics model expressing the next state in terms of the current state and the inputs
	Eigen::MatrixXd calcF(Eigen::VectorXd x, Eigen::VectorXd u, double dt); // the jacobian of the f functions wrt the state variables

	Eigen::MatrixXd V; // the process noise covariance matrix
	Eigen::MatrixXd W; // the observation noise covariance matrix

	Eigen::MatrixXd P; // the covariance matrix
	Eigen::MatrixXd K; // the kalman gain 

	double sign(double value);

public:
	ExtendedKalman(double _dt);
	int updateSensor(int index, double value);
	int updateControl(int index, double value);
	int updateState(double dt = -1.0);
};

using namespace Eigen;
int main(int argc, char** argv)
{
	//main6(argc, argv);

	ExtendedKalman ek(0.1);
	VectorXd x(12);
	char myvar[256];
	x << 23.0, 41.0, 33.0, 12.0, 14.0, -1.0, -3.0, -4.0, 0.05, 0.04, 50.0, 70.0;
	cout << x << endl;
	VectorXd u(1);
	u = VectorXd::Zero(1);
	VectorXd fnew = ek.calcf(x, u, 0.1);
	MatrixXd Fnew= ek.calcF(x, u, 0.1);
	cout << "f: \n" << fnew << endl;
	cout << "F: \n" << Fnew << endl;

	VectorXd hnew = ek.calch(x, u);
	MatrixXd Hnew = ek.calcH(x, u);
	cout << "h: \n" << hnew << endl;
	cout << "H: \n" << Hnew << endl;

	return 0;
}

int main1(int argc, char** argv)
{
	Mat image;
	Mat imgThresholded;
	Point blobCenter;

	VideoCapture cap(argv[1]);

	if (!cap.isOpened())
	{
		return -1;
	}

	Size videoSize(1280, 720);
	VideoWriter vidWriter("myVideo.avi", CV_FOURCC('M','J','P','G'), 30, videoSize);
	Mat videoFrame;

	while (cap.read(image))
	{
		

		//image = imread(argv[1], 1);
		//cap >> image;
		float scale = 2;
		resize(image, image, Size(int(image.cols / scale), int(image.rows / scale)));

		//threshold the image
		thresholdImage(image, imgThresholded);

		//detect the main blob
		blobCenter = detectBlobs(imgThresholded);

		//put a plus sign on the image at the blob center location
		line(image, Point(blobCenter.x + 20, blobCenter.y), Point(blobCenter.x - 20, blobCenter.y), CV_RGB(255, 0, 0),3);
		line(image, Point(blobCenter.x, blobCenter.y + 20), Point(blobCenter.x, blobCenter.y - 20), CV_RGB(255, 0, 0),3);
		//cout << "putting text" << endl;
		putText(image, "100 ft AGL", Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255, 0, 0), 3);

		namedWindow("Thresholded", 1);
		imshow("Thresholded", imgThresholded);

		namedWindow("Blob Location", 1);
		imshow("Blob Location", image);

		resize(image, videoFrame, videoSize);
		vidWriter.write(videoFrame);

		if (waitKey(30) == ' ')
		{
			cout << "writing image" << endl;
			imwrite("myImage.png", image);
			break;
		}
	}
	return 0;
	//get the video frame and detect the target dot
	//get the telemetry
	//calculate gimbal corrections
	//overlay telemetry on image and display
	//estimate the current pose
	//create package with commands to send to plane

}

Point detectBlobs(Mat image)
{
	vector<KeyPoint> keypoints;

	//set up the parameters for our blob detector
	SimpleBlobDetector::Params blobParams = SimpleBlobDetector::Params();
	blobParams.blobColor = 255; //extract light blobs
	blobParams.filterByColor = true;
	blobParams.filterByCircularity = false;
	blobParams.filterByArea = false;
	blobParams.filterByConvexity = false;
	blobParams.filterByConvexity = false;

	//create the blob detector
	static Ptr<SimpleBlobDetector> blobDetector = SimpleBlobDetector::create(blobParams);

	//detect the blobs in the image
	blobDetector->detect(image, keypoints);

	//find the biggest blob
	Point blobCenter;
	float blobSize = 0;
	for (auto x : keypoints)
	{
		//cout << x.pt << '\t' << x.size << endl;
		if (x.size > blobSize)
		{
			blobCenter = x.pt;
			blobSize = x.size;
		}
	}

	//return the location of the biggest blob
	return blobCenter;
}

void thresholdImage(Mat & image, Mat & imgThresholded)
{
	Mat imgHSV;

	cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	int iLowH = 0;
	int iLowS = 0;
	int iLowV = 0;// 190;
	int iHighH = 179;
	int iHighS = 255;//40;
	int iHighV = 255;

	Scalar meanVal, stDevVal;
	meanStdDev(imgHSV, meanVal, stDevVal);
	cout << meanVal << ", " << stDevVal << endl;

	iLowV = meanVal[2] + 2.0*stDevVal[2];

	int openSize = 4;
	int closeSize = 4;


	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
	
	//morphological closing (fill small holes in the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(closeSize, closeSize)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(closeSize, closeSize)));

	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(openSize, openSize)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(openSize, openSize)));
	
}

int talkToGimbal(int argc, char** argv)
{
	CSerial serial;
	
	serial.Open(4, 9600);

	Gimbal gimbal(serial);

	float pitch = -30.0;

	while (true)
	{
		if (gimbal.setAngle(pitch, 0.0, 0.0))
		{
			cout << "Success: " << pitch << endl;
		}
		else
		{
			cout << "Failure" << endl;
		}
		pitch = pitch + 1.0;
		if (pitch > 30.0)
		{
			pitch = -30.0;
		}
	}

	return 0;
}

int main6(int argc, char** argv)
{
	Mat image;
	Mat imgThresholded;
	Point blobCenter;

	int capDevice = 1;
	VideoCapture cap(capDevice);
	bool canDisplay = true;

	if (!cap.isOpened())
	{
		canDisplay = false;
	}
	else
	{
		canDisplay = true;
	}

	while (true)
	{

		if (canDisplay)
		{
			cap >> image;

			//threshold the image
			thresholdImage(image, imgThresholded);

			//detect the main blob
			blobCenter = detectBlobs(imgThresholded);

			//put a plus sign on the image at the blob center location
			line(image, Point(blobCenter.x + 20, blobCenter.y), Point(blobCenter.x - 20, blobCenter.y), CV_RGB(255, 0, 0), 3);
			line(image, Point(blobCenter.x, blobCenter.y + 20), Point(blobCenter.x, blobCenter.y - 20), CV_RGB(255, 0, 0), 3);
			//cout << "putting text" << endl;
			putText(image, "100 ft AGL", Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255, 0, 0), 3);

			namedWindow("Thresholded", 1);
			imshow("Thresholded", imgThresholded);

			namedWindow("Blob Location", 1);
			imshow("Blob Location", image);
		}

		char key = waitKey(10);
		if (key == 'q')
		{
			break;
		}
		else if (key == 'r')
		{
			cap.open(capDevice);
			if (!cap.isOpened())
			{
				cout << "couldn't find device: " << capDevice << endl;
				canDisplay = false;
			}
			else
			{
				cout << "found device: " << capDevice << endl;
				canDisplay = true;
			}
		}
		else if (key == 'n')
		{
			capDevice = (capDevice + 1) % 2;
			cap.open(capDevice);
			if (!cap.isOpened())
			{
				cout << "couldn't find device: " << capDevice << endl;
				canDisplay = false;
			}
			else
			{
				cout << "found device: " << capDevice << endl;
				canDisplay = true;
			}
		}
	}
	return 0;
	//get the video frame and detect the target dot
	//get the telemetry
	double groundspeed = 15;
	//calculate gimbal corrections
	//overlay telemetry on image and display
	//estimate the current pose
	//create package with commands to send to plane
}

int nextState(VectorXd xold, VectorXd &xnew, VectorXd u, VectorXd &z)
{
	//
}

using namespace Eigen;


double ExtendedKalman::sign(double val)
{
	if (val >= 0)
		return 1.0;
	else
		return -1.0;
}
VectorXd ExtendedKalman::calch(VectorXd x, VectorXd u) // the function expressing each sensor value in terms of the state variables
{
	VectorXd h = VectorXd::Zero(n);

	h(0) = x(0); // x position from GPS
	h(1) = x(1); // y position from GPS
	h(2) = tan(x(4) / x(3)); // heading from GPS
	h(3) = sqrt(pow(x(3), 2) + pow(x(4), 2)); // speed from GPS
	h(4) = x(2); // altitude from GPS
	h(5) = x(2); // altitude from barometer
	h(6) = sqrt(pow(x(3) - x(6), 2) + pow(x(4) - x(7), 2) + pow(x(5), 2)); // airspeed from pitot tube
	h(7) = x(8); // roll of the aircraft from IMU
	h(8) = x(9); // pitch of the aircraft from IMU
	h(9) = tan((x(4) - x(7)) / (x(3) - x(6))); // yaw of the aircraft from IMU
	h(10) = M_PI_2 - tan((x(4) - x(7)) / (x(3) - x(6))) - tan((x(11) - x(1)) / (x(10) - x(0))); // yaw angle to target from gimbal
	h(11) = tan(x(2) / sqrt(pow(x(11) - x(1), 2) + pow(x(10) - x(0), 2))); //pitch angle to target from gimbal

	return h;
}
MatrixXd ExtendedKalman::calcH(VectorXd x, VectorXd u) // the jacobian of the h vector wrt the state variables
{
	MatrixXd H = MatrixXd::Zero(m, n);

	//numerically approximate the jacobian by perturbing each state variable in turn and storing the results
	//calculate our baseline
	VectorXd baseline(n);
	VectorXd perturbed(n);
	baseline = calch(x, u);
	//for each state variable
	for (int i = 0; i < n; i++)
	{
		//set the step value;
		double step = 0.000001;
		//perturb the relevant state variable;
		x(i) += step;
		//calculate the physics with the perturbation
		perturbed = calch(x, u);
		//store the finite difference derivative in the appropriate column of F
		H.col(i) = (perturbed - baseline) / step;
		//restore the state variable
		x(i) -= step;
	}

	return H;
}
VectorXd ExtendedKalman::calcf(VectorXd x, VectorXd u, double dt) // the physics model expressing the next state in terms of the current state and the inputs
{
	cout << "m: " << m << endl;
	VectorXd f(m);
	f = VectorXd::Zero(m);
	double k1 = 1.0 / 2.0*1.225 * 1.2 * 5.5/(30/2.2);
	cout << "k1: " << k1 << endl;
	double a0L = -4.0*M_PI / 180.0;
	double g = 9.81; //acceleration due to gravity in m/s^2

	//double psi = atan2(x(4), x(3)); // yaw wrt north
	double vpa = sqrt(pow(x(3) - x(6), 2) + pow(x(4) - x(7), 2) + pow(x(5), 2)); //airspeed. needed to calculate lift
	cout << "vpa: " << vpa << endl;
	double ay = k1*vpa*vpa * (x(9) - a0L)*sin(x(8)); // horizontal radial acceleration due to lift, positive right
	cout << "ay: " << ay << endl;
	double az = -k1*vpa*vpa*(x(9) - a0L)*cos(x(8)) + g; // vertical radial accelration due to lift, positive down
	cout << "az: " << az << endl;
	double cospsi = x(4) / sqrt(x(3)*x(3) + x(4)*x(4));
	cout << "cospsi: " << cospsi << endl;
	double sinpsi = x(3) / sqrt(x(3)*x(3) + x(4)*x(4));
	cout << "sinpsi: " << sinpsi << endl;


	f(0) = x(0) + x(3)*dt + 0.5*ay*dt*dt*cospsi;
	f(1) = x(1) + x(4)*dt - 0.5*ay*dt*dt*sinpsi;
	f(2) = x(2) + x(5)*dt - 0.5*az*dt*dt;
	f(3) = x(3) + ay*cospsi*dt;
	f(4) = x(4) - ay*sinpsi*dt;
	f(5) = x(5) - az*dt;
	f(6) = x(6);
	f(7) = x(7);
	f(8) = x(8);
	f(9) = x(9);
	f(10) = x(10);
	f(11) = x(11);

	return f;
}
MatrixXd ExtendedKalman::calcF(VectorXd x, VectorXd u, double dt) // the jacobian of the f functions wrt the state variables
{
	MatrixXd F = MatrixXd::Zero(n, n);
	
	//numerically approximate the jacobian by perturbing each state variable in turn and storing the results
	//calculate our baseline
	VectorXd baseline(n);
	VectorXd perturbed(n);
	baseline = calcf(x, u, dt);
	//for each state variable
	for (int i = 0; i < n; i++)
	{
		//set the step value;
		double step = 0.000001;
		//perturb the relevant state variable;
		x(i) += step;
		//calculate the physics with the perturbation
		perturbed = calcf(x, u, dt);
		//store the finite difference derivative in the appropriate column of F
		F.col(i) = (perturbed - baseline) / step;
		//restore the state variable
		x(i) -= step;
	}

	return F;
}


/***************************************************************************
* This is the constructor. For this class, it doesn't do much because
* everything in the class has been fully defined.
***************************************************************************/
ExtendedKalman::ExtendedKalman(double _dt) 
{
	m_dt = _dt;
	V = MatrixXd::Zero(n, n); // Zero out the process noise matrix
	// Set the relevant variables in the process noise matrix
	W = MatrixXd::Zero(m, m); // Zero out the observation noise matrix
	// Set the relevant variables in the observation noise matrix
	P = MatrixXd::Ones(n, n) * 10000.0; // Set the covariance matrix to large numbers
	K = MatrixXd::Zero(n, m); // Zero out the kalman gains
	
	//initialize our state estimate
	x = MatrixXd::Zero(n, 1);
}

/***************************************************************************
* This function should be called whenever a sensor provides additional data.
***************************************************************************/
int ExtendedKalman::updateSensor(int index, double value)
{
	z(index, 1) = value;
	return 0;
}

/***************************************************************************
* This function should be called whenever a control input is sent
***************************************************************************/
int ExtendedKalman::updateControl(int index, double value)
{
	u(index, 1) = value;
	return 0;
}

/***************************************************************************
 * This function updates the state of the Kalman filter. It should be called 
 * as frequently as possible. If the time step is varying from 
 * call to call, then pass dt in as a parameter. If not, the last value input
 * will be used. This could be the value from the constructor.
 **************************************************************************/
int ExtendedKalman::updateState(double _dt)
{
	if (_dt > 0)
	{
		m_dt = _dt;
	}

	MatrixXd F(n,n);
	MatrixXd H(m, n);

	//xnew = f(xold,uold)
	x = calcf(x, u, m_dt);
	//Pnew = F(xnew,uold)*Pold*F(xnew,uold)' + V
	F = calcF(x, u, m_dt);
	P = F*P*F.transpose() + V;
	//K = Pnew*H(xnew,uold)'*(Hnew*Pnew*Hnew'+W)^-1
	H = calcH(x, u);
	K = (H * P * H.transpose() + W).transpose().colPivHouseholderQr().solve(H*P.transpose()).transpose();
	//xnew = xnew + Knew*(znew-h(xnew))
	x = x + K*(z - calch(x, u));
	//Pnew = (I-Knew*Hnew)*Pnew
	P = (MatrixXd::Identity(n, n) - K*H(x, u))*P;

	return 0;
}
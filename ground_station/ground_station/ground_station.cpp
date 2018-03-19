#define _USE_MATH_DEFINES

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <stdio.h>
#include "Serial.h"
#include "Gimbal.h"
#include <Eigen/Dense>
#include "ExtendedKalman.h"
#include <random>
#include <chrono>



using namespace std;
using namespace cv;
using namespace Eigen;

//Point detectBlobs(Mat image);
KeyPoint detectBlobs(Mat image, KeyPoint prevBlob);
void thresholdImage(Mat & image, Mat & imgThresholded);
int main1(int argc, char** argv);
int talkToGimbal(int argc, char** argv);
int main6(int argc, char** argv);
int listenToGimbal();
int trackDot(int argc, char** argv);
VectorXd getU(double t);

int main(int argc, char** argv)
{		
	trackDot(argc, argv);
	return 0;

	listenToGimbal();
	CSerial serial;
	serial.Open(6, 115200);
	Gimbal gimbal(serial);

	union Buf
	{
		char c[256] = "";
		uint8_t u[256];
	} outBuf,inBuf;

	gimbal.setAngle(0, 0, 0);
	//gimbal.requestAttitudeInterval(250000);
	int count = 0;
	while (true)
	{
		count++;
		if (count > 100000)
		{
			gimbal.setAngle(0, 0, 0);
			gimbal.requestAttitude();
			count = 0;
			printf("Pitch: \t%f\nRoll: \t%f\nYaw: \t%f\n", gimbal.gimbalPitch, gimbal.gimbalRoll, gimbal.gimbalYaw);
		}
		//gimbal.requestAttitude();
		gimbal.readMessage();
	}
	
	while (true)
	{
		while (serial.ReadDataWaiting() > 0)
		{
			serial.ReadData(inBuf.u, 1);
			printf("%x", inBuf.u[0]);
			//cout << inBuf.u[0];
		}
		printf("\n");
		while (serial.ReadDataWaiting() == 0)
			continue;
		
	}
	

	while (true)
	{
		cin >> outBuf.c;
		int numChars = strlen(outBuf.c);
		serial.SendData(outBuf.u, numChars);
		while (serial.ReadDataWaiting() < 16)
		{
		}
		while (serial.ReadDataWaiting() > 0)
		{
			serial.ReadData(inBuf.u, 1);
			if (inBuf.c[0] != 'o')
			{
				printf("%x", inBuf.u[0]);
				//cout << inBuf.c[0];
			}
		}
		cout << "finished" << endl;
		int numRead = serial.ReadData(inBuf.u, 256);
		cout << "numRead:\t" << numRead << endl;
		cout << "strlen: \t" << strlen(inBuf.c) << endl;
		cout << inBuf.c << endl;
	}


}

int listenToGimbal()
{
	float pitchDes = 0;
	CSerial serial;
	serial.Open(6, 115200);
	Gimbal gimbal(serial);

	union Buf
	{
		char c[256] = "";
		uint8_t u[256];
	} outBuf, inBuf;

	while (pitchDes != 182.3)
	{
		cin >> pitchDes;
		/*pitchDes += 5;
		if (pitchDes >= 30)
			pitchDes = -30;*/
		gimbal.setAngle(pitchDes, 0, 0);
		//gimbal.requestAttitude();

		//strcpy_s(outBuf.c, "v");
		//serial.SendData(outBuf.u, 1);
		while (serial.ReadDataWaiting() < 3)
		{
			continue;
		}
		
		//int numChars = serial.ReadData(inBuf.u, 255);
		//inBuf.c[numChars] = 0;
		//cout << "buffer:\t " << inBuf.c << endl;

		gimbal.requestAttitude();
		gimbal.readMessage();
		cout << "Roll:\t" << gimbal.gimbalRoll << endl;
		cout << "Yaw:\t" << gimbal.gimbalYaw << endl;
		cout << "Pitch:\t" << gimbal.gimbalPitch << endl;

	}
	return 0;
}

int testKalman(int argc, char** argv)
{
	//initialize a random number generator
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::normal_distribution<double> distribution(0.0, 1.0);
	
	//initialize the kalman filter
	double step = 0.01;
	ExtendedKalman ek(step);
	double t = 0;

	VectorXd u(2);
	VectorXd x0(12);
	x0 << -50.0, 20, 40, 10, 5, 0, 2.5, -2.5, 0, 0, 50, 70;
	cout << "xstart: " << x0 << endl;
	VectorXd x1(12);
	VectorXd z(12);
	VectorXd v(12);
	VectorXd w(12);

	ek.x << 5, 5, 10, 11, 11, 0, 0, 0, 0, 0, 45, 60;
	//ek.x << -50.0, 20, 40, 10, 5, 0, 2.5, -2.5, 0, 0, 0, 0;
	//while it has been less than 30 seconds
	int count = 0;
	while (t < 30.0)
	{
		//get the input to the vehicle
		u = getU(t);
		//calculate the predicted next state of the vehicle from the current state and inputs
		x1 = ek.calcf(x0, u);
		
		//add some process noise to the next state
		
		v << distribution(generator)*0.2,
			distribution(generator)*0.2,
			distribution(generator)*0.05,
			distribution(generator)*0.5,
			distribution(generator)*0.5,
			distribution(generator)*0.25,
			distribution(generator)*2.0,
			distribution(generator)*2.0,
			distribution(generator)*0.005,
			distribution(generator)*0.005,
			distribution(generator)*0.0,
			distribution(generator)*0.0;

		x1 += v;

		cout << "x1: " << x1 << endl;
		//convert the next state to measurements
		z = ek.calch(x1, u);
		//add some measurement noise
		w << distribution(generator)*0.2,
			distribution(generator)*0.2,
			distribution(generator)*0.01,
			distribution(generator)*1.0,
			distribution(generator)*0.3,
			distribution(generator)*0.5,
			distribution(generator)*2.0,
			distribution(generator)*0.005,
			distribution(generator)*0.005,
			distribution(generator)*0.005,
			distribution(generator)*0.01,
			distribution(generator)*0.01;
		z += w;
		cout << "z: " << z << endl;
		//update the kalman filter with the noisy measurements
		ek.z = z;
		ek.updateState();
		cout << "updated state" << endl;

		//display the error in the kalman state
		cout << "error: \n" << ek.x - x1 << endl;

		//store the next state as the current state
		x0 = x1;

		//increment time
		t += step;
		count++;

		if (count % 10 == 0)
		{
			int temp;
			cin >> temp;
		}
	}

	return 0;
}

VectorXd getU(double t)
{
	double phi = 30.0*M_PI / 180.0*sin(t * 2 * M_PI / 15.0);
	double theta = 2.5*M_PI / 180.0*sin(t * 2 * M_PI / 5.0) + 5.0*M_PI / 180.0;
	VectorXd u(2);
	u << phi, theta;
	return u;
}

//int main1(int argc, char** argv)
//{
//	Mat image;
//	Mat imgThresholded;
//	Point blob.pt;
//
//	VideoCapture cap(argv[1]);
//
//	if (!cap.isOpened())
//	{
//		return -1;
//	}
//
//	Size videoSize(1280, 720);
//	VideoWriter vidWriter("myVideo.avi", CV_FOURCC('M','J','P','G'), 30, videoSize);
//	Mat videoFrame;
//
//	while (cap.read(image))
//	{
//		
//
//		//image = imread(argv[1], 1);
//		//cap >> image;
//		float scale = 2;
//		resize(image, image, Size(int(image.cols / scale), int(image.rows / scale)));
//
//		//threshold the image
//		thresholdImage(image, imgThresholded);
//
//		//detect the main blob
//		blobCenter = detectBlobs(imgThresholded);
//
//		//put a plus sign on the image at the blob center location
//		line(image, Point(blobCenter.x + 20, blobCenter.y), Point(blobCenter.x - 20, blobCenter.y), CV_RGB(255, 0, 0),3);
//		line(image, Point(blobCenter.x, blobCenter.y + 20), Point(blobCenter.x, blobCenter.y - 20), CV_RGB(255, 0, 0),3);
//		//cout << "putting text" << endl;
//		putText(image, "100 ft AGL", Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255, 0, 0), 3);
//
//		namedWindow("Thresholded", 1);
//		imshow("Thresholded", imgThresholded);
//
//		namedWindow("Blob Location", 1);
//		imshow("Blob Location", image);
//
//		resize(image, videoFrame, videoSize);
//		vidWriter.write(videoFrame);
//
//		if (waitKey(30) == ' ')
//		{
//			cout << "writing image" << endl;
//			imwrite("myImage.png", image);
//			break;
//		}
//	}
//	return 0;
//	//get the video frame and detect the target dot
//	//get the telemetry
//	//calculate gimbal corrections
//	//overlay telemetry on image and display
//	//estimate the current pose
//	//create package with commands to send to plane
//
//}

KeyPoint detectBlobs(Mat image, KeyPoint prevBlob)
{
	vector<KeyPoint> keypoints;
	KeyPoint newBlob;

	//set up the parameters for our blob detector
	SimpleBlobDetector::Params blobParams = SimpleBlobDetector::Params();
	blobParams.blobColor = 255; //extract light blobs
	blobParams.filterByColor = true;
	blobParams.filterByCircularity = false;
	blobParams.filterByArea = false;
	blobParams.filterByConvexity = false;
	blobParams.filterByConvexity = false;

	//create the blob detector
	Ptr<SimpleBlobDetector> blobDetector = SimpleBlobDetector::create(blobParams);

	//detect the blobs in the image
	blobDetector->detect(image, keypoints);

	//find the best blob
	double minCost = 1000000000;

	for (auto blob : keypoints)
	{
		//cout << x.pt << '\t' << x.size << endl;
		//calculate fitness of current blob by comparing with previous blob
		int distance = sqrt(pow(blob.pt.x - prevBlob.pt.x, 2) + pow(blob.pt.y - prevBlob.pt.y, 2));
		int sizeDiff = abs(blob.size - prevBlob.size);
		double cost = 100*distance + 20*sizeDiff;
		if (cost < minCost)
		{
			newBlob = blob;
			minCost = cost;q
		}
	}

	//return the location of the biggest blob
	return newBlob;
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
	//cout << meanVal << ", " << stDevVal << endl;

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

int trackDot(int argc, char** argv)
{
	Mat image;
	Mat imgThresholded;
	Point blobCenter;
	int errorX = 0;
	int errorY = 0;
	int controlX = 0;
	int controlY = 0;
	double kpX = 10;
	double kpY = -1;

	KeyPoint blob;
	KeyPoint prevBlob;

	CSerial serial;
	serial.Open(6, 115200);
	Gimbal gimbal(serial);
	gimbal.setAngle(-45, 0, 0);

	int capDevice = 1;
	VideoCapture cap(capDevice);
	bool canDisplay = true;
	bool followDot = false;
	destroyAllWindows();
	namedWindow("Thresholded", 1);
	namedWindow("Blob Location", 1);

	if (!cap.isOpened())
	{
		canDisplay = false;
	}
	else
	{
		canDisplay = true;
		cap >> image;
		blob.pt = Point(image.size().width / 2.0, image.size().height / 2.0);
		blob.size = 20;
	}

	while (true)
	{

		if (canDisplay)
		{
			cap >> image;

			//threshold the image
			thresholdImage(image, imgThresholded);

			//detect the main blob
			blob = detectBlobs(imgThresholded, prevBlob);
			//blobCenter = Point(200, 200);

			if (followDot)
			{
				// get the error from image center to blob center
				errorX = image.size().width / 2.0 - blob.pt.x;
				errorY = image.size().height / 2.0 - blob.pt.y;

				//scale the error and add it to the current gimbal position to get the desired gimbal position
				controlX = kpX*errorX + 1500;
				controlY = kpY*errorY + 1500;

				//write the desired gimbal position to the gimbal.
				//gimbal.setPitchRollYaw(controlY, 1500, controlX);
				gimbal.setRCInputs(controlY, 0, controlX);

				prevBlob = blob;
			}

			//put a plus sign on the image at the blob center location
			line(image, Point(blob.pt.x + 20, blob.pt.y), Point(blob.pt.x - 20, blob.pt.y), CV_RGB(255, 0, 0), 3);
			line(image, Point(blob.pt.x, blob.pt.y + 20), Point(blob.pt.x, blob.pt.y - 20), CV_RGB(255, 0, 0), 3);
			//cout << "putting text" << endl;
			putText(image, "100 ft AGL", Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(255, 0, 0), 3);

			
			imshow("Thresholded", imgThresholded);

			
			imshow("Blob Location", image);
		}

		char key = waitKey(10);
		cout << "key: " << key << endl;
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
			cout << "switching to next device" << endl;
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
		else if (key == 'f')
		{
			followDot = !followDot;
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



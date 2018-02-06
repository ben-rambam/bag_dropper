#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <stdio.h>
#include "Serial.h"
#include "Gimbal.h"
#include <Eigen/Dense>

using namespace std;
using namespace cv;

Point detectBlobs(Mat image);
void thresholdImage(Mat & image, Mat & imgThresholded);
int main1(int argc, char** argv);
int main2(int argc, char** argv);
int main5(int argc, char** argv);
int main6(int argc, char** argv);
int main7(int argc, char** argv);

int main(int argc, char** argv)
{
	main7(argc, argv);
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

int main2(int argc, char** argv)
{
	VideoCapture cap(argv[1]); //capture the video from web cam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	int stDevs = 1;

	int closeSize = 3;
	int openSize = 3;

	//Create trackbars in "Control" window
	//cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	//cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("stDevs", "Control", &stDevs, 30); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	cvCreateTrackbar("closeSize", "Control", &closeSize, 14); //Value (0 - 255)
	cvCreateTrackbar("openSize", "Control", &openSize, 14);

	Mat imgOriginal;

	//imgOriginal = imread(argv[1], 1);
	int scalefactor = 3;
	//resize(imgOriginal, imgOriginal, Size(imgOriginal.cols / scalefactor, imgOriginal.rows / scalefactor));

	bool mode = 1;
	Mat imgHSV;
	Mat imgThresholded;

	while (true)
	{
		if (mode)
		{
			bool bSuccess = cap.read(imgOriginal); // read a new frame from video


			if (!bSuccess) //if not success, break loop
			{
				cout << "Cannot read a frame from video stream" << endl;
				break;
			}

			resize(imgOriginal, imgOriginal, Size(imgOriginal.cols / scalefactor, imgOriginal.rows / scalefactor));

			cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

			
		}


		
		// Threshold the image
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
		
																									  
		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(closeSize, closeSize)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(closeSize, closeSize)));

		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(openSize, openSize)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(openSize, openSize)));

		

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image

											/// Separate the image in 3 places ( B, G and R )
		vector<Mat> bgr_planes;
		split(imgHSV, bgr_planes);

		Scalar meanVal, stDevVal;
		meanStdDev(imgHSV, meanVal, stDevVal);
		cout << meanVal << ", " << stDevVal << endl;

		iLowV = meanVal[2] + float(stDevs) / 10.0*stDevVal[2];

		/// Establish the number of bins
		int histSize = 256;

		/// Set the ranges ( for B,G,R) )
		float range[] = { 0, 256 };
		const float* histRange = { range };

		bool uniform = true; bool accumulate = false;

		Mat b_hist, g_hist, r_hist;


		/// Compute the histograms:
		calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

		b_hist.at<float>(0) = 100;
		g_hist.at<float>(0) = 100;
		r_hist.at<float>(0) = 100;

		// Draw the histograms for B, G and R
		int hist_w = 512; int hist_h = 400;
		int bin_w = cvRound((double)hist_w / histSize);

		Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

		/// Normalize the result to [ 0, histImage.rows ]
		normalize(b_hist, b_hist, 0, hist_h, NORM_MINMAX, -1, Mat());
		normalize(g_hist, g_hist, 0, hist_h, NORM_MINMAX, -1, Mat());
		normalize(r_hist, r_hist, 0, hist_h, NORM_MINMAX, -1, Mat());

		/// Draw for each channel
		for (int i = 1; i < histSize; i++)
		{
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
				Scalar(255, 0, 0), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
				Scalar(0, 255, 0), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
				Scalar(0, 0, 255), 2, 8, 0);
		}

		/// Display
		namedWindow("calcHist Demo", WINDOW_AUTOSIZE);
		imshow("calcHist Demo", histImage);

		char key = 'a';
		key = waitKey(30);
		cout << key << endl;

		if (key == ' ')
		{
			mode = !mode;
			cout << mode << endl;
			cout << "pausing" << endl;
		}
		else if (key == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;

}


using namespace cv;

int main3(int argc, char** argv)
{
	Mat src, hsv;
	//if (argc != 2 || !(src = imread(argv[1], 1)).data)
	//	return -1;

	VideoCapture cap(0); //capture the video from web cam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	bool bSuccess = cap.read(src); // read a new frame from video

	if (!bSuccess) //if not success, break loop
	{
		cout << "Cannot read a frame from video stream" << endl;
		exit(1);
	}

	cvtColor(src, hsv, COLOR_BGR2HSV);

	// Quantize the hue to 30 levels
	// and the saturation to 32 levels
	int hbins = 30, sbins = 32;
	int histSize[] = { hbins, sbins };
	// hue varies from 0 to 179, see cvtColor
	float hranges[] = { 0, 180 };
	// saturation varies from 0 (black-gray-white) to
	// 255 (pure spectrum color)
	float sranges[] = { 0, 256 };
	const float* ranges[] = { hranges, sranges };
	MatND hist;
	// we compute the histogram from the 0-th and 1-st channels
	int channels[] = { 0, 1 };

	calcHist(&hsv, 1, channels, Mat(), // do not use mask
		hist, 2, histSize, ranges,
		true, // the histogram is uniform
		false);
	double maxVal = 0;
	minMaxLoc(hist, 0, &maxVal, 0, 0);

	int scale = 10;
	Mat histImg = Mat::zeros(sbins*scale, hbins * 10, CV_8UC3);

	for (int h = 0; h < hbins; h++)
		for (int s = 0; s < sbins; s++)
		{
			float binVal = hist.at<float>(h, s);
			int intensity = cvRound(binVal * 255 / maxVal);
			rectangle(histImg, Point(h*scale, s*scale),
				Point((h + 1)*scale - 1, (s + 1)*scale - 1),
				Scalar::all(intensity),
				CV_FILLED);
		}
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound((double)hist_w / hbins);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

	for (int i = 1; i < hbins; i++)
	{
		line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i))),
			Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
			Scalar(255, 0, 0), 2, 8, 0);
	}
	for (int i = 1; i < sbins; i++)
	{
		line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i))),
			Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
			Scalar(255, 0, 0), 2, 8, 0);
	}

	namedWindow("Source", 1);
	imshow("Source", src);

	namedWindow("H-S Histogram", 1);
	imshow("H-S Histogram", histImg);
	waitKey();
}



/**
* @function main
*/
int main4(int argc, char** argv)
{
	Mat src, dst, hsv;

	///// Load image
	//src = imread(argv[1], 1);

	VideoCapture cap(0); //capture the video from web cam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	while (true)
	{

		bool bSuccess = cap.read(src); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		cvtColor(src, hsv, COLOR_BGR2HSV);

		if (!hsv.data)
		{
			return -1;
		}

		/// Separate the image in 3 places ( B, G and R )
		vector<Mat> bgr_planes;
		split(hsv, bgr_planes);

		/// Establish the number of bins
		int histSize = 256;

		/// Set the ranges ( for B,G,R) )
		float range[] = { 0, 256 };
		const float* histRange = { range };

		bool uniform = true; bool accumulate = false;

		Mat b_hist, g_hist, r_hist;

		/// Compute the histograms:
		calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

		// Draw the histograms for B, G and R
		int hist_w = 512; int hist_h = 400;
		int bin_w = cvRound((double)hist_w / histSize);

		Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

		/// Normalize the result to [ 0, histImage.rows ]
		normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

		/// Draw for each channel
		for (int i = 1; i < histSize; i++)
		{
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
				Scalar(255, 0, 0), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
				Scalar(0, 255, 0), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
				Scalar(0, 0, 255), 2, 8, 0);
		}

		/// Display
		namedWindow("calcHist Demo", WINDOW_AUTOSIZE);
		imshow("calcHist Demo", histImage);
		namedWindow("original", WINDOW_AUTOSIZE);
		imshow("original", src);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;
}

int main5(int argc, char** argv)
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
	double 
	//calculate gimbal corrections
	//overlay telemetry on image and display
	//estimate the current pose
	//create package with commands to send to plane
}

int main7(int argc, char **argv)
{
	VideoCapture cap(argv[1]);
	if (!cap.isOpened())
	{
		cout << "problem opening file" << endl;
		return -1;
	}

	Mat image;
	int frame = 0;

	namedWindow("image");
	cap >> image;
	cout << frame << endl;
	while (true)
	{
		imshow("image", image);
		char key = waitKey(10);
		if (key == 'n')
		{
			cap >> image;
			frame++;
			cout << frame << endl;
		}
		if (key == 'q')
			break;
	}
}
#include<iostream>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define MIN_FEAS 200
#define FastConst 30
#define termCrit TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01)

float focal;
float speed = 0.01; //Speed of the camera

Point2d pp;

void detectFeatures(Mat inputImg, vector<Point2f> &points);

void tractFeatures(Mat prevImg, vector<Point2f> &prevPts, Mat currImg, vector<Point2f> &currPts, vector<uchar> &status);

void processFrame(Mat frame, Mat prevImg, Mat R, Mat T, vector<Point2f> &prevFeatures);


int main(int argc, char **argv)
{
	std::cout << "maiin";
	VideoCapture cap;
	cap.open("/home/basestation/output.avi");
	cap.set(CV_CAP_PROP_POS_FRAMES, 60);


	std::cout << "entered";

	if (!cap.isOpened())
	{
		std::cout << "Camera is not open!!!!\n";
		return 0;
	}

	Mat T_f, R_f; //kameranın güncel konum ve rotasyon matrisi
	Mat R, T; //kameranın anlık değişen konum ve rotasyonu

	//yörüngeyi gösterecek resim
	Mat traj = Mat::zeros(600, 600, CV_8UC3);

	Mat prevImg;
	vector<Point2f> prevFeatures;	

	Mat img, img2;

	cap.read(img);
	cap.read(img2);

	float w = img.cols;
	float h = img.rows;

	float cx = w/2;
	float cy = h/2;

	pp = Point2d(cx, cy);
	focal = 14.8;

	cvtColor(img, prevImg, COLOR_BGR2GRAY);
	
	detectFeatures(prevImg, prevFeatures);
	processFrame(img2, prevImg, R, T, prevFeatures);

	T_f = T.clone();
	R_f = R.clone();

	int done_frames = 0;

	std::cout << "Began";

	while(1)
	{
		if (!cap.isOpened())
		{
			std::cout << "Camera is not open!!!!\n";
			break;
		}

		/*
		done_frames++;
		if(done_frames % 2 != 0)
			continue;
		*/

		Mat frame;
		cap.read(frame);

		//odometry algoritması devreye girer
		processFrame(frame, prevImg, R, T, prevFeatures);

		if (waitKey(10) == 27) //27 == esc
			break;

		imshow("video", frame);
		imshow("traj", traj);

		if (1)//((speed > 0) &&(T.at<double>(2) > T.at<double>(1)) && (T.at<double>(2) > T.at<double>(0)))
		{
			T_f = T_f + speed * (R_f * T);
			R_f = R * R_f;
		}
		else
		{
			cout << "GECERSIZ HAREKET \n";
			continue;
		}

		int x = int(T_f.at<double>(0)) + 300;
		int y = int(T_f.at<double>(2)) + 100;
		circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
	}

	return 0;
}


void detectFeatures(Mat inputImg, vector<Point2f> &points)
{
	std::vector<KeyPoint> keyPoints;
	FAST(inputImg, keyPoints, FastConst, true);

	//ileriki asama için "keypoint" türü "point2f" türüne dönüþtürülmeli,
	KeyPoint::convert(keyPoints, points, vector<int>());
}

void tractFeatures(Mat prevImg, vector<Point2f> &prevPts, Mat currImg, vector<Point2f> &currPts, vector<uchar> &status)
{
	std::vector<float> err;
	
	calcOpticalFlowPyrLK(prevImg, currImg, prevPts, currPts, status, err, Size(21, 21), 3, termCrit, 0, 0.001);

	int indexCorrection = 0;
	for (int i = 0; i < status.size(); i++)
	{
		Point2f pt = currPts.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
			if ((pt.x < 0) || (pt.y < 0)) {
				status.at(i) = 0;
			}
			currPts.erase(currPts.begin() + (i - indexCorrection));
			prevPts.erase(prevPts.begin() + (i - indexCorrection));
			indexCorrection++;
		}
	}
}

void processFrame(Mat frame, Mat prevImg, Mat R, Mat T, vector<Point2f> &prevFeatures)
{
	Mat currImg, currG;
	//alýnan resmi siyah-beyaz formata dönüþtür
	cvtColor(frame, currImg, COLOR_BGR2GRAY);
	//undistort(currG, currImg, mtx, dist);

	vector<Point2f> currFeatures;
	vector<uchar>status;

	for (auto& p : prevFeatures)
	{
		circle(frame, p, 2, Scalar(0, 0, 255), 1, 8);
	}

	std::cout << prevFeatures.size();

	Mat mask;
	tractFeatures(prevImg, prevFeatures, currImg, currFeatures, status);
	
	Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, currFeatures, prevFeatures, R, T, focal, pp, mask);

	//zaman geçtikçe özellik sayýsý düþecektir
	//eðer özellik sayýsý belli bir sýnrýn altýna düþer ise tekrardan arama yapacaðýz
	if (prevFeatures.size() < MIN_FEAS)
	{
		detectFeatures(prevImg, prevFeatures);
		tractFeatures(prevImg, prevFeatures, currImg, currFeatures, status);
	}

	prevImg = currImg.clone();
	prevFeatures = currFeatures;
}
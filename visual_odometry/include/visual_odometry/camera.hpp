#include<opencv2/opencv.hpp>
#include<vector>

using namespace cv;

#define MIN_FEAS 500
#define FastConst 10

class Camera
{
	TermiCriteria termCrit TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
	float focal;
	Point2d pp;
	VideoCapture cap;
	Mat prevImg, dist, mtx;
	std::vector<Point2f> prevFeatures;
	bool first = true;

	Mat getFrame();
	void processFrame(Mat frame, Mat prevImg, Mat R, Mat T, std::vector<Point2f> &prevFeatures);
	void detectFeatures(Mat inputImg, std::vector<Point2f> &points);
	void tractFeatures(Mat prevImg, std::vector<Point2f> &prevPts, Mat currImg, std::vector<Point2f> &currPts, std::vector<uchar> &status);

public:
	Mat T, R;

	Camera(Mat _mtx, Mat _dist);
	void update(float speed);s
}

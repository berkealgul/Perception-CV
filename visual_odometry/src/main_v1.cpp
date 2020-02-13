#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

#define MIN_FEAS 500
#define FastConst 10


class Camera
{
	//TermiCriteria termCrit TermCriteria::TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
	Point2d pp;
	VideoCapture cap;
	Mat prevImg, dist, mtx;
	std::vector<Point2f> prevFeatures;
	float focal;
	bool first = true;

	Mat getFrame()
	{
	    Mat rawFrame, grayFrame, finalFrame;

	    cap.read(rawFrame);
	    cvtColor(rawFrame, grayFrame, COLOR_BGR2GRAY);
	    undistort(finalFrame, grayFrame, mtx, dist);

	    return finalFrame;
	}

	void processFrame(Mat frame, Mat prevImg, Mat R, Mat T, std::vector<Point2f> &prevFeatures)
	{
		Mat currImg;
		//alýnan resmi siyah-beyaz formata dönüþtür
		std::vector<Point2f> currFeatures;
		std::vector<uchar>status;

		Mat mask;

		tractFeatures(prevImg, prevFeatures, currImg, currFeatures, status);

		Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R, T, focal, pp, mask);

		if (prevFeatures.size() < MIN_FEAS)
		{
			detectFeatures(prevImg, prevFeatures);
			tractFeatures(prevImg, prevFeatures, currImg, currFeatures, status);
		}

		prevImg = currImg.clone();
		prevFeatures = currFeatures;
	}

	void detectFeatures(Mat inputImg, std::vector<Point2f> &points)
	{
		std::vector<KeyPoint> keyPoints;
		FAST(inputImg, keyPoints, FastConst, true);

		//ileriki aþama için "keypoint" türü "point2f" türüne dönüþtürülmeli,
		KeyPoint::convert(keyPoints, points, std::vector<int>());
	}

	void tractFeatures(Mat prevImg, std::vector<Point2f> &prevPts, Mat currImg, std::vector<Point2f> &currPts, std::vector<uchar> &status)
	{
		std::vector<float> err;
		calcOpticalFlowPyrLK(prevImg, currImg, prevPts, currPts, status, err,
            Size(21, 21), 3, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), 0, 0.001);

		//hatalý eþleþmelerden kurtulur lakin
		//bu kýsým bana gizemli geldi o yüzden ayrýntýlar ile ilgili bir þey diyemeyeceðim
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


public:
	Mat T, R;

	Camera(Mat _mtx, Mat _dist)
	{
	    cap = VideoCapture(0);
	    mtx = _mtx;
	    dist = _dist;

	    Mat frame;
	    cap.read(frame);

	    float w = frame.cols;
	    float h = frame.rows;
	    float fx = mtx.at<double>(0,0);
	    float cx = mtx.at<double>(0,2);
	    float cy = mtx.at<double>(1,2);

	    pp = Point2d(cx, cy);
	    focal = fx / w;
	}

	Mat update(float speed)
	{
        Mat frame = getFrame();

	    if(speed <= 0.1)
	    {
	        std::cout << "Low Speed\n";
	        return frame;
	    }

	    Mat dR, dT;

	    if (first == true)
	    {
	        detectFeatures(prevImg, prevFeatures);
		    processFrame(frame, prevImg, dR, dT, prevFeatures);
	        first = false;

	    }
        else
        {
            processFrame(frame, prevImg, dR, dT, prevFeatures);
        }

	    if ((T.at<double>(2) > T.at<double>(1)) && (T.at<double>(2) > T.at<double>(0)))
	    {
	        T = T + speed * (R * dT);
	    	R = dR * R;
	    }
	    else
	        std::cout << "Invalid movement\n";
	}
};


float speed = 0; //Speed of the camera

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    float ax = imu->linear_acceleration.x;
    float ay = imu->linear_acceleration.y;
    float az = imu->linear_acceleration.z;

    speed += sqrt(ax*ax+ay*ay+az*az);
    std::cout << speed << "\n";
}

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_odometry_v1");

    Mat_<double> mtx(3,3);
    Mat_<double> dist(1,5);

    dist << 0.020436355102596344, -0.11407839179793304, 0.004229887050454093, -0.01709654130034178, 0.13991605472148272;
    mtx << 627.2839475395182, 0.0, 295.0153571445745,
           0.0, 630.6046803340988, 237.10098847214766,
           0.0, 0.0, 1.0;

    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/visual_odom_pose", 10);
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);

    Camera cam(mtx, dist);

    while (ros::ok())
    {
        Mat frame = cam.update(speed);
        imshow("frame", frame);
        ros::spinOnce();

        //TODO:Add pose publisher
    }

	return 0;

}

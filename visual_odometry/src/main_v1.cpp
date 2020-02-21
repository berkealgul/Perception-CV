#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>

using namespace cv;
using namespace std;

#define MIN_FEAS 800
#define FastConst 20


// TODO : Consider speed's position in code
float speed = 0; //Speed of the camera
float sx = 0;
float sy = 0;
float sz = 0;


class Camera
{
 public:
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
	    undistort(grayFrame, finalFrame, mtx, dist);

	    return finalFrame;
	}

	void processFrame(Mat currImg, Mat prevImg, Mat R, Mat T)
	{
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

	void drawFeatures(Mat src)
	{
		for (int i = 0; i < prevFeatures.size(); ++i)
        {	
        	Point2f pf = prevFeatures[i];
        	Point p(int(pf.x), int(pf.y));
        	circle(src, p, 1, Scalar::all(-1));
        }	
	}

 //public:
	Mat T, R;

	Camera(Mat _mtx, Mat _dist)
	{
	    cap.open(1);
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

	void update(float speed)
	{	

        Mat frame = getFrame();

        drawFeatures(frame);
		imshow("frame", frame);

	    if(speed <= 0.1)
	    {
	        std::cout << "Low Speed: " << speed << "\n";
	        return;
	    }

	    Mat dR, dT;

	    if (first == true)
	    {  	
	        detectFeatures(prevImg, prevFeatures);
		    processFrame(frame, prevImg, dR, dT);
	        first = false;
	    }
        else
        {
            processFrame(frame, prevImg, dR, dT);
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

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    float ax = imu->linear_acceleration.x;
    float ay = imu->linear_acceleration.y;
    float az = imu->linear_acceleration.z;

    sx += ax;
    sy += ay;
    sz += az;

    speed += sqrt(sx*sx+sy*sy+sz*sz);
    //std::cout << speed << "\n";
}

void getQuaternion(Mat R, geometry_msgs::Quaternion Q)
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
 
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q.w = (s * 0.5);
        s = 0.5 / s;
        Q.x = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q.y = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q.z = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q.x = s * 0.5;
        s = 0.5 / s;

        Q.w = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q.y = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q.z = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}

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
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/visual_odom_path", 10);
	ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);
	ros::Time time;	
	nav_msgs::Path path;

    Camera cam(mtx, dist);

    while (ros::ok())
    {
    	ros::spinOnce();
    	/*
    	Mat frame, gray;
    	cap.read(frame);
    	std::vector<KeyPoint> keyPoints;
    	cvtColor(frame, gray, COLOR_BGR2GRAY);
		FAST(gray, keyPoints, FastConst, true);
		drawKeypoints(gray, keyPoints, frame);
    	imshow("frane", frame);
    	int key = waitKey(10);
    	continue;
    	*/

    	//TODO: Draw circle to frame in order to indicate detected features on frame 
    	//Mat frame;
        cam.update(speed);
        int key = waitKey(30);
        continue;
        //Draw features

        //Construct and publish calculated data

        time = ros::Time::now();

        geometry_msgs::Pose pose;
        geometry_msgs::PoseStamped poseStamped;
        geometry_msgs::Quaternion q;
        geometry_msgs::Point p;

	    Mat T = cam.T;
	    Mat R = cam.R;
	   	
	   	// define required headers
	    path.header.stamp = time;
	 	path.header.frame_id = "vodom";
	    poseStamped.header.stamp = time;
	 	poseStamped.header.frame_id = "vodom";
	 	
	 	// Fill pose
	 	p.x = T.at<double>(0,0);
	 	p.y = T.at<double>(1,0);
	 	p.z = T.at<double>(2,0);
	 	getQuaternion(R, q);
	 	pose.position = p;
	 	pose.orientation = q;

	 	// fill pose for poseStamped
	 	poseStamped.pose = pose;

	 	// update path
	 	path.poses.push_back(poseStamped);

        path_pub.publish(path);

        //int key = waitKey(10);     
    }

	return 0;
}

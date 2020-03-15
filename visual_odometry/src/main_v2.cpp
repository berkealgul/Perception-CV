#include<iostream>
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

#define MIN_FEAS 200
#define FastConst 5
#define termCrit TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01)

float focal;
float speed = 0.001; //Speed of the camera
float sx = 0;
float sy = 0;
float sz = 0;

Point2d pp;

Mat_<double> mtx(3,3);
Mat_<double> dist(1,5);

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

void processFrame(Mat frame, Mat prevImg, Mat R, Mat T, vector<Point2f> &prevFeatures)
{
	Mat currImg, currG;
	//alýnan resmi siyah-beyaz formata dönüþtür
	cvtColor(frame, currG, COLOR_BGR2GRAY);
	undistort(currG, currImg, mtx, dist);

	vector<Point2f> currFeatures;
	vector<uchar>status;

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

void getQuaternion(Mat R, double Q[])
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
 
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[0] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[2] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[1] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    float ax = imu->linear_acceleration.x;
    float ay = imu->linear_acceleration.y;
    float az = imu->linear_acceleration.z;

    sx += ax;
    sy += ay;
    sz += az;

    speed = sqrt(sx*sx+sy*sy+sz*sz);
    //std::cout << speed << "\n";
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "visual_odometry_v2");
    dist << 0.020436355102596344, -0.11407839179793304, 0.004229887050454093, -0.01709654130034178, 0.13991605472148272;
    mtx << 627.2839475395182, 0.0, 295.0153571445745,
           0.0, 630.6046803340988, 237.10098847214766,
           0.0, 0.0, 1.0;


    ros::NodeHandle nh;
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/visual_odom_path", 10);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/visual_odom_pose", 10);
	ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);
	ros::Time time;	
	nav_msgs::Path path;

	VideoCapture cap;
	cap.open(1);

	Mat T_f, R_f; //kameranın güncel konum ve rotasyon matrisi
	Mat R, T; //kameranın anlık değişen konum ve rotasyonu

	//yörüngeyi gösterecek resim
	Mat traj = Mat::zeros(600, 600, CV_8UC3);

	Mat prevImg;
	vector<Point2f> prevFeatures;


	//döngüye girmeden tanımlama işlemleri yapılmalı

	Mat img, img2, imgG;

	cap.read(img);
	cap.read(img2);

	float w = img.cols;
	float h = img.rows;
	float fx = mtx.at<double>(0,0);
	float cx = mtx.at<double>(0,2);
	float cy = mtx.at<double>(1,2);

	pp = Point2d(cx, cy);
	focal = fx / w;

	cvtColor(img, imgG, COLOR_BGR2GRAY);
	undistort(imgG, prevImg, mtx, dist);

	detectFeatures(prevImg, prevFeatures);
	processFrame(img2, prevImg, R, T, prevFeatures);

	T_f = T.clone();
	R_f = R.clone();

	char filename[200];

	while(ros::ok())
	{
		ros::spinOnce();

		if (!cap.isOpened())
		{
			std::cout << "Camera is not open!!!!\n";
			continue;
		}

		Mat frame;
		cap.read(frame);

		//odometry algoritması devreye girer
		processFrame(frame, prevImg, R, T, prevFeatures);

		//eğer geçerli bir hareket elde ediyor isek ve hareket boyutu düşük değilse
		//kameranın mutlak konum ve rotasyonu hesapla
		if ((speed > 0.1) &&(T.at<double>(2) > T.at<double>(1)) && (T.at<double>(2) > T.at<double>(0)))
		{
			T_f = T_f + speed * (R_f * T);
			R_f = R * R_f;
		}
		else
		{
			cout << "GECERSIZ HAREKET \n";
			continue;
		}
		

		time = ros::Time::now();

        geometry_msgs::Pose pose;
        geometry_msgs::PoseStamped poseStamped;
        geometry_msgs::Quaternion q;
        geometry_msgs::Point p;
	   	
	   	// define required headers
	    path.header.stamp = time;
	 	path.header.frame_id = "map";
	    poseStamped.header.stamp = time;
	 	poseStamped.header.frame_id = "map";
	 	
	 	// Fill pose
	 	p.x = T_f.at<double>(0,0);
	 	p.y = T_f.at<double>(1,0);
	 	p.z = T_f.at<double>(2,0);
	 	double Q[4];
	 	getQuaternion(R_f, Q);

	 	q.x = Q[0];
	 	q.y = Q[1];
	 	q.z = Q[2];
	 	q.w = Q[3];

	 	//std::cout << q;

	 	pose.position = p;
	 	pose.orientation = q;

	 	// fill pose for poseStamped
	 	poseStamped.pose = pose;
	 	pose_pub.publish(poseStamped);

	 	path.poses.push_back(poseStamped);
        path_pub.publish(path);

		//resimleri göster
		imshow("video", frame);

		cout << T_f << std::endl;
		cout << prevFeatures.size();
		cout << "\n\n";    

		if (waitKey(1) == 27) //27 == esc
			break;
	}

	return 0;
}

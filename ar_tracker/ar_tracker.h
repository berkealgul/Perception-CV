#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "definitions.h"

#define PI 3.14159265358979323846
#define FIXED_FRAME "aruco_slam_world"

using namespace cv;
using namespace aruco;

class ArTracker
{
public:

	//aruco stoff
	Ptr<DetectorParameters> params;
	Ptr<Dictionary> dictionary;

	//opencv stuff
	VideoCapture cap;
	Mat_<double> mtx(3,3);
	Mat_<double> dist(1,5);

	//ros stuff
	ros::NodeHandle n;
	tf::TransformBroadcaster br;

	//tree stuff
	slam_tree tree;

	//custom stuff
	int iter;
	bool showFrame;

	//functions
	ArTracker();

	void step();
	void relocalize_marker(slam_obj* marker, slam_obj* camera, tf::Quaternion R_cm, tf::Vector3 T_cm);
	void broadcast_tf(tf::TransformBroadcaster br, slam_obj *obj);
	void update_camera_transform(slam_obj* camera, std::vector<tf::Quaternion> qv, std::vector<tf::Vector3> tv);

	std::tuple<tf::Quaternion, tf::Vector3> estimate_cam_transform(tf::Quaternion R_cm, tf::Vector3 T_cm ,slam_obj *camera, slam_obj *marker);
	slam_obj *create_marker_obj(tf::Quaternion R_cm, tf::Vector3 T_cm, int id, slam_obj* camera);
	slam_obj *create_camera();	
};

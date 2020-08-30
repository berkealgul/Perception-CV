#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace Eigen;
using namespace cv;

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloudConstPtr &cloud_msg)
{
	sensor_msgs::PointCloud2 cloud2_msg;	
	sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud2_msg);

	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(cloud2_msg, *cloud);
  	

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize(0.1, 0.1, 0.1);
	sor.filter(cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(*cloud, output);

	pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
	pcl::fromPCLPointCloud2(cloud_filtered, cloud_xyz);

	// Publish the data
	pub.publish(output);
}

float interpolate(float x0, float x1, float y0, float y1, float x)
{
	float y;
	float w = (x-x0)/(x1-x0);
	y = y0*(1-w) + y1*w;
	return y;
}


int main(int argc, char** argv)
{
	ros::init (argc, argv, "camera_lidar_fusion");
	
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/pointcloud", 1000, cloud_cb);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/voxelcloud", 1);

	Matrix4f ext(4,4);
	ext << 1,0,0,0,
		   0,1,0,0,
		   0,0,1,0,
		   0,0,0,1;

	MatrixXf ins(3,4);
	ins << 517.3, 0.0, 320, 0.0,
		   0.0, 516.5, 240, 0.0,
		   0.0, 0.0, 0.0, 1.0;

	MatrixXf pro = ins * ext;

	Mat img = imread("/home/basestation/test_ws/src/test/src/image.png",IMREAD_COLOR);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/basestation/test_ws/src/test/src/cloud.pcd", *cloud) == -1) //* load the file
    {
    	PCL_ERROR ("Couldn't read pcd \n");
    	return (-1);
    }

    MatrixXf cloud_mat = cloud->getMatrixXfMap(4, 4, 0);
	MatrixXf img_mat = pro * cloud_mat;

	int w = img.cols;
	int h = img.rows;

	// max rgb = 7, 237, 245
	float maxR = 7;
	float maxG = 237;
	float maxB = 245;
	// min rgb = 242, 30, 2
	float minR = 242;
	float minG = 30;
	float minB = 2;

	for(int i = 0, nCols = img_mat.cols(); i < nCols; i++)
	{
		int u = (int)img_mat(0,i);
		int v = (int)img_mat(1,i);

		// if point projected outside of the image
		// ignore it
		if(u < 0 || u >= w || v < 0 || v >= h)
		{
			continue;
		}

		float d = cloud_mat(2, i);

		float r = interpolate(1, 4, minR, maxR, d);
		float g = interpolate(1, 4, minG, maxG, d);
		float b = interpolate(1, 4, minB, maxB, d);

		circle(img, Point(u,v), 2, Scalar(b,g,r),1);
		//imshow("g", img);	
		//waitKey(100);
		std::cout << d << "\n";
	}

	imwrite("result.png", img);
	return 0;
}
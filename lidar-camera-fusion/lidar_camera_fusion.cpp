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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

using namespace Eigen;
using namespace cv;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_lidar_fusion");

	//KITTI K00 canfig 

	MatrixXf velo_to_cam(3,4);
	velo_to_cam << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03, 
				   1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
				   9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01;

	MatrixXf cam_to_px(3,3);
	cam_to_px << 7.215377e+02, 0.000000e+00, 6.095593e+02, 
		 		 0.000000e+00, 7.215377e+02, 1.728540e+02, 
		 		 0.000000e+00, 0.000000e+00, 1.000000e+00;

	// final projection matrix
	MatrixXf P = cam_to_px * velo_to_cam;

	Mat img = imread("/home/basestation/KITTI/image.png",IMREAD_COLOR);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/basestation/KITTI/cloud.pcd", *cloud) == -1) //* load the file
    {
    	PCL_ERROR ("Couldn't read pcd \n");
    	return (-1);
    }

    MatrixXf cloud_mat = cloud->getMatrixXfMap(4, 4, 0);

    // final projected pixel coordinates
	MatrixXf img_mat = P * cloud_mat;

	int w = img.cols;
	int h = img.rows;

	//Depth map for depth values
	ArrayXXf depth_map = ArrayXXf::Zero(h, w);

	for(int i = 0, nCols = img_mat.cols(); i < nCols; i++)
	{	
		float z = img_mat(2,i);
		int u = img_mat(0,i)/z;
		int v = img_mat(1,i)/z;

		// if point projected outside of the image or z is non positive
		// ignore it
		if(u < 0 || u >= w || v < 0 || v >= h || z <= 0)
		{
			continue;
		}

		depth_map(v, u) = z;

		// rendering purposes not essensial
		float b = z / 20  * 120;
		float g = z / 10  * 120;
		float r = z / 5  * 120;

		circle(img, Point(u,v), 2, Scalar(b,g,r) ,1);

		//imshow("g", img);	
		//waitKey(100);
		
	}

	imwrite("result.png", img);
	
	/*
	// demo purposes
	pcl::visualization::PCLVisualizer viewer("lol");
	viewer.addPointCloud(cloud,"velo");
  	viewer.spin();
	*/
	return 0;
}
#include "ar_tracker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_slam");
	
	ArTracker ar;

	while(ros::ok())
		ar.step();

}
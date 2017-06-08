#include <ros/ros.h>
#include "cnbiros_bci/TidInterface.hpp"


int main(int argc, char** argv) {


	ros::init(argc, argv, "example_tid");
	ros::NodeHandle node;
	ros::Rate r(10);
	cnbiros::bci::TidInterface tid(&node);

	ROS_INFO("Try to connect to the cnbiloop...");
	tid.Connect();

	tid.Attach("/bus");	
	tid.Attach("/dev");	

	tid.Run();


	return 0;
}

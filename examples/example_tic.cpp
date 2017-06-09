#include <ros/ros.h>
#include "cnbiros_bci/TicInterface.hpp"


int main(int argc, char** argv) {


	ros::init(argc, argv, "example_tic");
	ros::NodeHandle node;
	ros::Rate r(10);
	cnbiros::bci::TicInterface tic(&node);

	ROS_INFO("Try to connect to the cnbiloop...");
	tic.Connect();

	tic.Attach("/ctrl1", cnbiros::bci::TicInterface::ToRos);	
	tic.Attach("/ctrl2", cnbiros::bci::TicInterface::ToCnbi);	
	//tic.Attach("/dev");	

	tic.Run();


	return 0;
}

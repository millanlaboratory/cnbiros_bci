#include <ros/ros.h>
#include "cnbiros_bci/TicInterface.hpp"


int main(int argc, char** argv) {
	
	// ros initialization
	ros::init(argc, argv, "tic");
	ros::NodeHandle node("~");
	
	// cnbiros initialization
	cnbiros::bci::TicInterface tic(&node);
	std::vector<std::string> pipes2ros;
	std::vector<std::string> pipes2cnbi;
	bool exit = false;

	if(ros::param::get("tic_pipes2ros", pipes2ros) == false &&
	   ros::param::get("tic_pipes2cnbi", pipes2cnbi) == false) {
		ROS_ERROR("No TiC pipes to cnbi/ros provided. Exit");
		ros::shutdown();
	}

	// Connection to cnbi loop (blocking)
	ROS_INFO("Try to connect to the cnbiloop...");
	tic.Connect();

	// Try to attach to the provided pipes2ros
	for(auto it = pipes2ros.begin(); it != pipes2ros.end(); ++it) {
		if(tic.Attach((*it), cnbiros::bci::TicInterface::ToRos) == false) {
			ROS_ERROR("Cannot attach to %s (to ROS). Exit", (*it).c_str());
			exit = true;
		} else {
			ROS_INFO("Attached to TiC %s (to ROS)", (*it).c_str());
		}
	}
	
	// Try to attach to the provided pipes2cnbi
	for(auto it = pipes2cnbi.begin(); it != pipes2cnbi.end(); ++it) {
		if(tic.Attach((*it), cnbiros::bci::TicInterface::ToCnbi) == false) {
			ROS_ERROR("Cannot attach to %s (to CNBI). Exit", (*it).c_str());
			exit = true;
		} else {
			ROS_INFO("Attached to TiC %s (to CNBI)", (*it).c_str());
		}
	}

	if(exit == false) {
		ROS_INFO("Listening to CNBI TiC pipes and ROS topics");
		tic.Run();
	}

	return 0;
}

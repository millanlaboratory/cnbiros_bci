#include <ros/ros.h>
#include "cnbiros_bci/TicInterface.hpp"


int main(int argc, char** argv) {
	
	// ros initialization
	ros::init(argc, argv, "rostic");
	ros::NodeHandle node("~");
	
	// cnbiros initialization
	cnbiros::bci::TicInterface tic(&node);
	std::vector<std::string> pipes2ros;
	std::vector<std::string> pipes2cnbi;

	// get custom parameters (mandatory) 
	node.getParam("/rostic/pipes2ros",  pipes2ros);
	node.getParam("/rostic/pipes2cnbi", pipes2cnbi);
	
	// Connection to cnbi loop (blocking)
	ROS_INFO("Try to connect to the cnbiloop...");
	tic.Connect();

	// Try to attach to the provided pipes2ros
	for(auto it = pipes2ros.begin(); it != pipes2ros.end(); ++it) {
		if(tic.Attach((*it), cnbiros::bci::TicInterface::ToRos) == false) {
			ROS_ERROR("Cannot attach to %s (to Ros). Exit", (*it).c_str());
		}
	}
	
	// Try to attach to the provided pipes2cnbi
	for(auto it = pipes2cnbi.begin(); it != pipes2cnbi.end(); ++it) {
		if(tic.Attach((*it), cnbiros::bci::TicInterface::ToCnbi) == false) {
			ROS_ERROR("Cannot attach to %s (to CNBI). Exit", (*it).c_str());
		}
	}

	if(pipes2ros.empty() == false || pipes2cnbi.empty() == false) {
		ROS_INFO("Listening to CNBI TiC pipes and ROS topics");
		tic.Run();
	} else {
		ROS_ERROR("Not provided any pipes to ROS or to CNBI loop. Exit.");
	}

	return 0;
}

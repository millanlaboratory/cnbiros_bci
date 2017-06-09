#include <ros/ros.h>
#include "cnbiros_bci/TidInterface.hpp"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "rostid");
	ros::NodeHandle node("~");
	
	// cnbiros initialization
	cnbiros::bci::TidInterface tid(&node);
	std::vector<std::string> pipes = {"/bus", "/dev"};
	bool exit = false;

	// get custom parameters (if exists, otherwise use the default)
	node.getParam("/rostid/pipes", pipes);

	// Connection to cnbi loop (blocking)
	ROS_INFO("Try to connect to the cnbiloop...");
	tid.Connect();

	// Attach to the provided pipes
	for(auto it = pipes.begin(); it != pipes.end(); ++it) {
		if(tid.Attach((*it)) == false) {
			ROS_ERROR("Cannot attach to %s. Exit", (*it).c_str());
			exit = true;
		}
	}

	if(exit == false) {
		ROS_INFO("Listening to CNBI TiD pipes and ROS topics");
		tid.Run();
	}

	node.shutdown();

	return 0;
}

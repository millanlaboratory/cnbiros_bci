#include <ros/ros.h>
#include "cnbiros_bci/TidInterface.hpp"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "rostid");
	ros::NodeHandle node;
	
	// cnbiros initialization
	cnbiros::bci::TidInterface* tid;
	std::string nameserver;
	std::vector<std::string> pipes;
	bool exit = false;

	// Getting nameserver address
	if(node.getParam("/rostid/nameserver", nameserver) == false) {
		ROS_INFO("Default nameserver address: 127.0.0.1:8123 (or the one in ~/.cnbiloop)");
	}
	
	// get custom parameters (if exists, otherwise use the default)
	//if(ros::param::get("tid_pipes", pipes) == false) {
	if(node.getParam("/rostid/tid_pipes", pipes) == false) {
		ROS_WARN("No Tid pipes to cnbi/ros provided. Use node services to add them.");
	}

	// Instanciate TicInterface
	tid = new cnbiros::bci::TidInterface(&node, nameserver);
	
	// Connection to cnbi loop (blocking)
	ROS_INFO("Try to connect to the cnbiloop...");
	tid->Connect();

	// Attach to the provided pipes
	for(auto it = pipes.begin(); it != pipes.end(); ++it) {
		if(tid->Attach((*it)) == false) {
			ROS_ERROR("Cannot attach to %s. Exit", (*it).c_str());
		}
	}

	if(exit == false) {
		ROS_INFO("Listening to CNBI TiD pipes and ROS topics");
		tid->Run();
	}

	delete tid;

	return 0;
}

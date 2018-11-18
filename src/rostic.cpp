#include <ros/ros.h>
#include "cnbiros_bci/TicInterface.hpp"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "rostic");
	
	// cnbiros initialization
	cnbiros::bci::TicInterface tic;

	tic.Run();

	return 0;
}

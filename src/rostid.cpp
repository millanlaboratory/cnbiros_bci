#include <ros/ros.h>
#include "cnbiros_bci/TidInterface.hpp"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "rostid");
	
	// cnbiros initialization
	cnbiros::bci::TidInterface tid;

	tid.Run();

	return 0;
}

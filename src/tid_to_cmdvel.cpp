#include <ros/ros.h>
#include "cnbiros_bci/TidToCmdVel.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "tid_to_cmdvel");
	
	cnbiros::bci::TidToCmdVel t;
	ros::spin();
	
	return 0;
}

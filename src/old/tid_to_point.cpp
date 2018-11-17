#include <ros/ros.h>
#include "cnbiros_bci/TidToPoint.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "tid_to_point");
	
	cnbiros::bci::TidToPoint t;
	ros::spin();
	
	return 0;
}

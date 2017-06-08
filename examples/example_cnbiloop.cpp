#include <ros/ros.h>
#include "cnbiros_bci/TobiInterface.hpp"


int main(int argc, char** argv) {


	ros::init(argc, argv, "example_cnbiloop");
	ros::NodeHandle node;
	ros::Rate r(10);
	cnbiros::bci::TobiInterface cnbiloop;
	

	while(node.ok()) {


		ROS_INFO("Try to connect to the cnbiloop...");
		cnbiloop.Connect();

		while(cnbiloop.IsConnected()) {
			ROS_INFO_ONCE("Loop connected... doing stuff");
			
			r.sleep();
			ros::spinOnce();
		}
		ROS_WARN_ONCE("Loop disconnected");
	}



	return 0;
}

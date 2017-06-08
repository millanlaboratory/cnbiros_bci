#include <ros/ros.h>
#include "cnbiros_bci/TobiInterface.hpp"
#include "cnbiros_bci/TidClientSet.hpp"


int main(int argc, char** argv) {


	ros::init(argc, argv, "example_cnbiloop");
	ros::NodeHandle node;
	ros::Rate r(10);
	cnbiros::bci::TobiInterface cnbiloop;
	cnbiros::bci::TidClientSet tidset;


	ROS_INFO("Try to connect to the cnbiloop...");
	cnbiloop.Connect();

	tidset.Add("/bus", ClTobiId::GetOnly);
	tidset.Add("/dev", ClTobiId::SetOnly);

	ClTobiId* id;

	while(node.ok() & cnbiloop.IsConnected()) {

		if(tidset.Get("/bus", id))
			if(id->IsAttached() == false)
				id->Attach("/bus");

		if(tidset.Get("/dev", id))
			if(id->IsAttached() == false)
				id->Attach("/dev");
			
		r.sleep();
		ros::spinOnce();
	}


	return 0;
}

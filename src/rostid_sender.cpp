#include <tobiid/IDMessage.hpp>
#include <ros/ros.h>

#include "cnbiros_bci/TidInterface.hpp"

void usage(void) { 
	printf("Usage: ros_tidsender [OPTION]...\n\n");
	printf("It publishes on %s topic a TiD Message\n\n", CNBIROS_BCI_TID_ROS2CNBI);
	printf("  -n       CNBI loop pipe name (/bus default)\n");
	printf("  -e       CNBI GDF event (666 default)\n");
	printf("  -h       display this help and exit\n");
}

int main(int argc, char** argv) {

	int opt;
	std::string optpipe("/bus");
	std::string optevent("666");
	
	while((opt = getopt(argc, argv, "n:e:h")) != -1) {
		if(opt == 'n')
			optpipe.assign(optarg);
		else if(opt == 'e')
			optevent.assign(optarg);
		else {
			usage();
			exit(opt == 'h' ? EXIT_SUCCESS : EXIT_FAILURE);
		}
	}

	// ros initialization
	ros::init(argc, argv, "ros_tidsender");
	ros::NodeHandle node;
	ros::Rate r(10);

	// Set ROS publisher
	ros::Publisher rospub = node.advertise<cnbiros_bci::TidMessage>(CNBIROS_BCI_TID_ROS2CNBI, 1000);


	// Initialize the message
	cnbiros_bci::TidMessage 	rosidm;
	unsigned int rosevent = std::stoi(optevent);

	rosidm.header.stamp    = ros::Time::now();
	rosidm.header.frame_id = CNBIROS_BCI_TID_FRAMEID;
	rosidm.version         = CNBIROS_BCI_TID_VERSION;
	rosidm.family 		   = IDMessage::FamilyBiosig;
	rosidm.description 	   = "ros_tidsender";
	rosidm.event 		   = rosevent;
	rosidm.pipe			   = optpipe;

	ROS_INFO("Press 'enter' to send GDF=%u, 'q' to quit:", rosevent);
	while(node.ok()) {
		
		printf("NDF=-1 >> ");
		switch(getchar()) {
			case 'q':
				ros::shutdown();
				break;
			default:
				break;
		}

		rospub.publish(rosidm);

		ros::spinOnce();
	}



	return 0;
}

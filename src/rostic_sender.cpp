#include <ros/ros.h>
#include "cnbiros_bci/TicInterface.hpp"

void usage(void) { 
	printf("Usage: ros_ticsender [OPTION]...\n\n");
	printf(" 	   It publishes on %s topic a default (sine) TiC Message\n\n", CNBIROS_BCI_TIC_ROS2CNBI);
	printf("  -n       CNBI loop pipe name (/ctrl1 default)\n");
	printf("  -h       display this help and exit\n");
}

float sinewave(float a, float dt, float f) {
	return a*sin((2.0f*M_PI*dt*f));
}


int main(int argc, char** argv) {

	int opt;
	std::string optname("/ctrl1");
	
	while((opt = getopt(argc, argv, "n:h")) != -1) {
		if(opt == 'n')
			optname.assign(optarg);
		else {
			usage();
			exit(opt == 'h' ? EXIT_SUCCESS : EXIT_FAILURE);
		}
	}

	// ros initialization
	ros::init(argc, argv, "ros_ticsender");
	ros::NodeHandle node("~");
	ros::Rate r(10);

	// Set ROS publisher
	ros::Publisher rospub = node.advertise<cnbiros_bci::TicMessage>(CNBIROS_BCI_TIC_ROS2CNBI, 1000);
	

	// Initialize the message
	cnbiros_bci::TicMessage 	rosicm;
	cnbiros_bci::TicClassifier 	rosicc;
	cnbiros_bci::TicClass 		rosicl;
	
	rosicl.label = "6100";
	rosicl.value = 0.0f;

	rosicc.name 	   = "ros_ticsender";
	rosicc.description = "ROS dummy classifier";
	rosicc.vtype 	   = ICClassifier::ValueProb;
	rosicc.ltype 	   = ICClassifier::LabelBiosig;
	rosicc.classes.push_back(rosicl);

	rosicm.header.stamp    = ros::Time::now();
	rosicm.header.frame_id = "base_link";
	rosicm.version 		   = "0.0.1";
	rosicm.frame 		   = -1;
	rosicm.pipe 		   = optname;
	rosicm.classifier 	   = rosicc;
	
	double dt;
	ros::Time begin = ros::Time::now();

	ROS_INFO("Start publishing TiC message on %s", CNBIROS_BCI_TIC_ROS2CNBI);
	while(node.ok()) {
		
		rosicc.classes.clear();	
	
		dt = (ros::Time::now().toSec() - begin.toSec());
		rosicl.value = sinewave(1.0f, dt, 0.1);
		rosicc.classes.push_back(rosicl);
		
		rosicm.classifier = rosicc;

		rospub.publish(rosicm);

		r.sleep();
		ros::spinOnce();
	}




	return 0;
}

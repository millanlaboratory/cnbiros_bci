#ifndef CNBIROS_BCI_TIDTOCMDVEL_HPP
#define CNBIROS_BCI_TIDTOCMDVEL_HPP

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Package includes
#include "cnbiros_bci/TidMessage.h"

namespace cnbiros {
	namespace bci {

class TidToCmdVel {

	public:
		TidToCmdVel(void);
		virtual ~TidToCmdVel(void);

		virtual bool configure(void);

	private:
		void on_received_tid(const cnbiros_bci::TidMessage& msg);
		float rad2deg(float rad);
		float deg2rad(float deg);

	private: 
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber sub_;
		ros::Publisher  pub_;
		std::string		stopic_;
		std::string		ptopic_;

		std::string		frame_id_;

		std::map<std::string, std::string> cmd_labels_;
		std::map<std::string, geometry_msgs::Twist> cmd_vel_;
		std::map<std::string, bool> cmd_mode_;

		geometry_msgs::Twist	current_cmd_;


};

	}
}


#endif

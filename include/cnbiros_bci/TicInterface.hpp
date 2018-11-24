#ifndef CNBIROS_BCI_TICINTERFACE_HPP
#define CNBIROS_BCI_TICINTERFACE_HPP

#include <ros/ros.h>
#include <tobiic/ICMessage.hpp>
#include <cnbiloop/ClTobiIc.hpp>

#include "cnbiros_bci/TobiInterface.hpp"
#include "cnbiros_bci/TicTools.hpp"
#include "cnbiros_tobi_msgs/TicMessage.h"

namespace cnbiros {
	namespace bci {

class TicInterface : public TobiInterface {
	
	public:
		TicInterface(void);
		~TicInterface(void);

		bool configure(void);

		bool Attach(void);
		bool Detach(void);
		bool IsAttached(void);
		bool ReAttach(void);

		bool Run(void);

	protected:
		void on_received_ros2tic(const cnbiros_tobi_msgs::TicMessage& msg);

	private:
		void init_tobiic(void);
		void destroy_tobiic(void);

	private:
		ros::NodeHandle	nh_;
		ros::NodeHandle	p_nh_;
		std::string		nname_;
		ros::Subscriber sub_;
		ros::Publisher  pub_;
		std::string		stopic_;
		std::string		ptopic_;
		
		std::string			pipe_;
		ClTobiIc*			tobiic_  = nullptr;

		bool	has_ros_message_;
		ICMessage						fromLoopMsg_;
		cnbiros_tobi_msgs::TicMessage	fromRosMsg_;
		ICMessage						toLoopMsg_;
		cnbiros_tobi_msgs::TicMessage	toRosMsg_;

};

	}
}

#endif

#ifndef CNBIROS_BCI_TICINTERFACE_HPP
#define CNBIROS_BCI_TICINTERFACE_HPP

#include <ros/ros.h>
#include <tobiic/ICMessage.hpp>

#include <cnbiros_core/SetSubscribers.hpp>
#include <cnbiros_core/SetPublishers.hpp>

#include "cnbiros_bci/TobiInterface.hpp"
#include "cnbiros_bci/TicClientSet.hpp"
#include "cnbiros_bci/TicTools.hpp"
#include "cnbiros_bci/TicMessage.h"

#define CNBIROS_BCI_TIC_CNBI2ROS 	"/tic_cnbi2ros"
#define CNBIROS_BCI_TIC_ROS2CNBI 	"/tic_ros2cnbi"

namespace cnbiros {
	namespace bci {

class TicInterface : public TobiInterface {
	
	public:
		TicInterface(ros::NodeHandle* node, CcAddress address = "127.0.0.1:8123");
		~TicInterface(void);

		bool Attach(const std::string& pipe, unsigned int mode);
		void Detach(const std::string& pipe);

		void Run(void);

	public:
		static const unsigned int ToRos  = 0;
		static const unsigned int ToCnbi = 1;

	private:
		void callback_ros2tic(const cnbiros_bci::TicMessage& msg);

	private:
		TicClientSet*			ticclset_;
		core::SetPublishers* 	pubset_;
		core::SetSubscribers* 	subset_;
		ros::NodeHandle* 		rosnode_;


};

	}
}

#endif

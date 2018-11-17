#ifndef CNBIROS_BCI_TIDINTERFACE_HPP
#define CNBIROS_BCI_TIDINTERFACE_HPP

#include <ros/ros.h>
#include <tobiid/IDMessage.hpp>
#include <cnbiloop/ClTobiId.hpp>

#include "cnbiros_bci/TobiInterface.hpp"
//#include "cnbiros_bci/TidTools.hpp"
#include "cnbiros_tobi_msgs/TidMessage.h"

namespace cnbiros {
	namespace bci {

class TidInterface : public TobiInterface {
	
	public:
		TidInterface(void);
		virtual ~TidInterface(void);

		bool Attach(void);
		bool Detach(void);

		void Run(void) {};

	private:
		void on_received_ros2tid(const cnbiros_tobi_msgs::TidMessage& msg);

	private:
		ClTobiId*		tobiid_ = nullptr;
		ros::Subscriber sub_;
		ros::Publisher  pub_;
		std::string		stopic_;
		std::string		ptopic_;






};

	}
}

#endif

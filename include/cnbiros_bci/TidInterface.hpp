#ifndef CNBIROS_BCI_TIDINTERFACE_HPP
#define CNBIROS_BCI_TIDINTERFACE_HPP

#include <ros/ros.h>
#include <tobiid/IDMessage.hpp>

#include <cnbiros_core/Subscribers.hpp>
#include <cnbiros_core/Publishers.hpp>

#include "cnbiros_bci/Flags.hpp"
#include "cnbiros_bci/TobiInterface.hpp"
#include "cnbiros_bci/TidClientSet.hpp"
#include "cnbiros_bci/TidTools.hpp"
#include "cnbiros_tobi_msgs/TidMessage.h"

#include "cnbiros_bci/SetTid.h"
#include "cnbiros_bci/UnSetTid.h"

namespace cnbiros {
	namespace bci {

class TidInterface : public TobiInterface {
	
	public:
		TidInterface(ros::NodeHandle* node, CcAddress address = "127.0.0.1:8123");
		~TidInterface(void);

		bool Attach(const std::string& pipe);
		bool Detach(const std::string& pipe);

		void Run(void);

	private:
		void callback_ros2tid(const cnbiros_tobi_msgs::TidMessage& msg);
		bool on_set_tid_(cnbiros_bci::SetTid::Request &req,
						 cnbiros_bci::SetTid::Response &res);
		bool on_unset_tid_(cnbiros_bci::UnSetTid::Request &req,
						   cnbiros_bci::UnSetTid::Response &res);

	private:
		TidClientSet*		tidclset_;
		core::Publishers* 	pubset_;
		core::Subscribers* 	subset_;
		ros::NodeHandle* 	rosnode_;
		ros::ServiceServer	rossrv_set_tid_;
		ros::ServiceServer	rossrv_unset_tid_;


};

	}
}

#endif

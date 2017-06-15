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
#include "cnbiros_bci/TidMessage.h"

namespace cnbiros {
	namespace bci {

class TidInterface : public TobiInterface {
	
	public:
		TidInterface(ros::NodeHandle* node, CcAddress address = "127.0.0.1:8123");
		~TidInterface(void);

		bool Attach(const std::string& pipe);
		void Detach(const std::string& pipe);

		void Run(void);

	private:
		void callback_ros2tid(const cnbiros_bci::TidMessage& msg);

	private:
		TidClientSet*			tidclset_;
		core::Publishers* 	pubset_;
		core::Subscribers* 	subset_;
		ros::NodeHandle* 		rosnode_;


};

	}
}

#endif

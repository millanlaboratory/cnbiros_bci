#ifndef CNBIROS_BCI_TICINTERFACE_HPP
#define CNBIROS_BCI_TICINTERFACE_HPP

#include <ros/ros.h>
#include <tobiic/ICMessage.hpp>

#include <cnbiros_core/Subscribers.hpp>
#include <cnbiros_core/Publishers.hpp>

#include "cnbiros_bci/Flags.hpp"
#include "cnbiros_bci/TobiInterface.hpp"
#include "cnbiros_bci/TicClientSet.hpp"
#include "cnbiros_bci/TicTools.hpp"
#include "cnbiros_bci/TicMessage.h"

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
		core::Publishers* 	pubset_;
		core::Subscribers* 	subset_;
		ros::NodeHandle* 		rosnode_;


};

	}
}

#endif

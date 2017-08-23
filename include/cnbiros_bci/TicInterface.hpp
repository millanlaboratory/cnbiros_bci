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

#include "cnbiros_bci/SetTic.h"
#include "cnbiros_bci/UnSetTic.h"
#include "cnbiros_bci/Sync.h"

namespace cnbiros {
	namespace bci {

class TicInterface : public TobiInterface {
	
	public:
		TicInterface(ros::NodeHandle* node, CcAddress address = "127.0.0.1:8123");
		~TicInterface(void);

		bool Attach(const std::string& pipe, unsigned int mode);
		bool Detach(const std::string& pipe);

		void Run(void);

	public:
		static const unsigned int ToRos  = 0;
		static const unsigned int ToCnbi = 1;

	private:
		void callback_ros2tic(const cnbiros_bci::TicMessage& msg);
		bool on_set_tic(cnbiros_bci::SetTic::Request &req,
						 cnbiros_bci::SetTic::Response &res);
		bool on_unset_tic(cnbiros_bci::UnSetTic::Request &req,
						   cnbiros_bci::UnSetTic::Response &res);
		bool on_sync(cnbiros_bci::Sync::Request &req,
					     cnbiros_bci::Sync::Response &res);

	private:
		TicClientSet*		ticclset_;
		core::Publishers* 	pubset_;
		core::Subscribers* 	subset_;
		ros::NodeHandle* 	rosnode_;
		ros::ServiceServer	rossrv_set_tic_;
		ros::ServiceServer	rossrv_unset_tic_;
		ros::ServiceServer	rossrv_sync_;
		ICMessage*			cnbiicm_;
		ICSerializerRapid*	cnbiics_;
		int 				syncidx_;

};

	}
}

#endif

#ifndef CNBIROS_BCI_TICINTERFACE_CPP
#define CNBIROS_BCI_TICINTERFACE_CPP

#include "cnbiros_bci/TicInterface.hpp"

namespace cnbiros {
	namespace bci {

TicInterface::TicInterface(void) : p_nh_("~") {

	this->stopic_ = "rostic_ros2cnbi";
	this->ptopic_ = "rostic_cnbi2ros";


	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &TicInterface::on_received_ros2tic, this);
	this->pub_ = this->nh_.advertise<cnbiros_tobi_msgs::TicMessage>(this->ptopic_, 1);

	this->nname_ = ros::this_node::getName();
}

TicInterface::~TicInterface(void) {
	this->Detach();
	if(this->tobiic_ != nullptr)
		delete this->tobiic_;
}

bool TicInterface::configure(void) {

	bool retcode = true;
	
	// Getting parameters
	this->p_nh_.getParam("loopip", this->loop_ip_);
	this->p_nh_.getParam("reconnect", this->rtime_);
	this->p_nh_.getParam("pipe", this->pipe_);
	this->p_nh_.getParam("mode", this->nmode_);

	// Check interface mode
	if(!this->nmode_.compare("SetGet")) {
		ROS_ERROR("[%s] - %s not allowed for TobiIc. Aborting.", 
				  this->nname_.c_str(), this->nmode_.c_str());
		return false;
	}
	
	// Set interface mode
	if(this->SetMode(this->nmode_) == false) {
		ROS_ERROR("[%s] - Unknown interface mode provided (%s). Aborting.", 
				  this->nname_.c_str(), this->nmode_.c_str());
		retcode = false;
	}

	// Check if the pipe is provided
	if(this->pipe_.empty() == true) {
		ROS_ERROR("[%s] - No cnbi pipe is provided. Aborting.", this->nname_.c_str());
		retcode = false;
	}

	// Instantiate cltobiic with the correct mode
	this->tobiic_  = new ClTobiIc(this->GetMode());

	return retcode;
}

bool TicInterface::Attach(void) {

	if(this->IsAttached())
		return true;
		
	if(this->tobiic_->Attach(this->pipe_)) {
		ROS_INFO("[%s] - Attached to %s pipe as %s", 
				 this->nname_.c_str(), this->pipe_.c_str(),
				 this->GetModeName().c_str());
	}

	return this->IsAttached();
}


bool TicInterface::Detach(void) {
	bool retcode = false;

	if(this->IsAttached() == false)
		return true;
		
	if(this->tobiic_->Detach())
		ROS_WARN("[%s] - Detached from %s pipe", this->nname_.c_str(), this->pipe_.c_str());
	
	return !(this->IsAttached());
}

bool TicInterface::IsAttached(void) {

	bool ret = false;
	if(this->tobiic_ != nullptr)
		ret = this->tobiic_->IsAttached();

	return ret;
}

void TicInterface::on_received_ros2tic(const cnbiros_tobi_msgs::TicMessage& msg) {
	ROS_DEBUG("[%s] - Received TiC message on topic: %s", this->nname_.c_str(), this->stopic_.c_str());
	this->fromRosMsg_ = msg;
	this->has_ros_message_ = true;
}

bool TicInterface::Run(void) {

	ICMessage						toLoopMsg;
	cnbiros_tobi_msgs::TicMessage	toRosMsg;
	ICSerializerRapid				sloop(&(this->fromLoopMsg_));
	ICSerializerRapid				sros(&(toLoopMsg));

	TicTools	tictool;
	ros::Rate r(50);

	// Configuration
	if(this->configure() == false) {
		ROS_ERROR("[%s] - Node configuration failed", this->nname_.c_str());
		return false;
	}

	// Connection to cnbi loop
	if(this->Connect() == false) {
		ROS_ERROR("[%s] - Cannot connect to cnbi loop", this->nname_.c_str());
		return false;
	}

	// Main loop
	while(this->nh_.ok()) {

		// Attaching to cnbi loop
		if(this->Attach() == false) {
			ROS_ERROR_THROTTLE(5.0f, "[%s] - Cannot attach to %s pipe", this->nname_.c_str(), this->pipe_.c_str());
		} 

		// Getting ic message from loop (GetOnly)
		if(this->GetMode() == TobiInterface::GetOnly) {
			switch(this->tobiic_->GetMessage(&sloop)) {
				case ClTobiIc::Detached:
					ROS_WARN("[%s] - Connection on pipe %s detached. Try to attach again..", 
							      this->nname_.c_str(), this->pipe_.c_str());
					break;
				case ClTobiIc::NoMessage:
					break;
				case ClTobiIc::HasMessage:
					ROS_INFO_ONCE("[%s] - First message received from pipe %s", 
							      this->nname_.c_str(), this->pipe_.c_str());
					toRosMsg = TicTools::ToRos(this->fromLoopMsg_, this->pipe_);
					this->pub_.publish(toRosMsg);
					break;
			}
		// Getting ic message from ros (SetOnly)
		} else if(this->GetMode() == TobiInterface::SetOnly) {
			if(this->IsAttached() && this->has_ros_message_ == true) {
				ROS_INFO("[%s] - Received TiC message from ros", this->nname_.c_str());

				//try {
				//	toLoopMsg = tictool.GetMessage(this->fromRosMsg_);
				//	this->tobiic_->SetMessage(&sros);
				//} catch (TCException& e) {
				//	printf("%s\n", e.GetInfo().c_str());
				//}
				this->has_ros_message_ = false;
			}

		}



		// Getting id message from ros

		ros::spinOnce();
		r.sleep();
	}
}
/*
void TicInterface::Run(void) {

	//ICMessage 		  		cnbiIcm;
	//ICSerializerRapid 		cnbiIcs(&cnbiIcm);
	TicTools 		  		tictool;
	std::vector<cnbiros_tobi_msgs::TicMessage> rosIcmList;
	ros::Rate r(50);
	bool newmessage = false;
	while(this->rosnode_->ok()) {

		for(auto it = this->ticclset_->Begin(); it != this->ticclset_->End(); ++it) {
			if(it->second->GetMode() == ClTobiIc::GetOnly) {
		
				switch(it->second->GetMessage(this->cnbiics_)) {
					case ClTobiIc::Detached:
						ROS_WARN("Connection on pipe %s detached. Try to attach again..", it->first.c_str());
						if(it->second->Attach(it->first)) {
							ROS_INFO("Connection on pipe %s attached", it->first.c_str());
						}
						break;
					case ClTobiIc::NoMessage:
						break;

					case ClTobiIc::HasMessage:
						ROS_INFO_ONCE("First message received from pipe %s", it->first.c_str());
						if(this->cnbiicm_->GetBlockIdx() >= this->syncidx_) {
							rosIcmList = tictool.GetMessage(*(this->cnbiicm_), it->first);
							for(auto itm = rosIcmList.begin(); itm != rosIcmList.end(); ++itm) {
								this->pubset_->Publish(CNBIROS_BCI_TIC_CNBI2ROS, (*itm));
							}
						}

				}


				////if(it->second->WaitMessage(&cnbiIcs) == ClTobiIc::HasMessage) {
				//if(it->second->GetMessage(this->cnbiics_) == ClTobiIc::HasMessage) {
				//	ROS_INFO_ONCE("First message received from pipe %s", it->first.c_str());
				//	//rosIcmList = tictool.GetMessage(cnbiIcm, it->first);
				//	rosIcmList = tictool.GetMessage(*(this->cnbiicm_), it->first);
				//	for(auto itm = rosIcmList.begin(); itm != rosIcmList.end(); ++itm) {
				//		this->pubset_->Publish(CNBIROS_BCI_TIC_CNBI2ROS, (*itm));
				//	}

				////} else if(it->second->GetMessage(&cnbiIcs) == ClTobiIc::Detached) {
				//} else if(it->second->GetMessage(this->cnbiics_) == ClTobiIc::Detached) {
				//	ROS_WARN("Connection on pipe %s detached. Try to attach again..", it->first.c_str());
				//	if(it->second->Attach(it->first)) {
				//		ROS_INFO("Connection on pipe %s attached", it->first.c_str());
				//	}
				//}
			}
		}
	
		r.sleep();
		ros::spinOnce();
	}
}



void TicInterface::callback_ros2tic(const cnbiros_tobi_msgs::TicMessage& rosIcm) {

	std::shared_ptr<ClTobiIc>   ptic;
	bool 		retcod = false;

	ICMessage		  		cnbiIcm;
	ICSerializerRapid 	cnbiIcs(&cnbiIcm);
	TicTools 				tictool;

	// Convert ROS message to TicMessage
	cnbiIcm = tictool.GetMessage(rosIcm);

	// Check if the corresponding ClTobiIc exists and is attached
	if(this->ticclset_->Get(rosIcm.pipe, ptic)) {
		if(ptic->IsAttached()) {
			ROS_INFO_ONCE("First message streamed from ROS to pipe %s", rosIcm.pipe.c_str());
			ptic->SetMessage(&cnbiIcs);
			retcod = true;
		} else {	
			ROS_WARN_THROTTLE(5, "Connection on pipe %s detached. Try to attach again..", rosIcm.pipe.c_str());
			if(ptic->Attach(rosIcm.pipe)) {
				ROS_INFO("Connection on pipe %s attached", rosIcm.pipe.c_str());
			}
		}
	}
	
	if(retcod == false) {
		ROS_ERROR_THROTTLE(10, "Message received from ROS to CNBILoop. " 
			      "However, no connection to pipe %s has been found.", rosIcm.pipe.c_str());	
	}

}
*/

	}
}

#endif

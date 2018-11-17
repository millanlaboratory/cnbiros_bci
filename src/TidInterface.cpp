#ifndef CNBIROS_BCI_TIDINTERFACE_CPP
#define CNBIROS_BCI_TIDINTERFACE_CPP

#include "cnbiros_bci/TidInterface.hpp"

namespace cnbiros {
	namespace bci {

TidInterface::TidInterface(void) : p_nh_("~") {

	this->stopic_ = "rostid_ros2cnbi";
	this->ptopic_ = "rostid_cnbi2ros";

	this->tobiid_ = new ClTobiId(this->GetMode());

	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &TidInterface::on_received_ros2tid, this);
	this->pub_ = this->nh_.advertise<cnbiros_tobi_msgs::TidMessage>(this->ptopic_, 1);

	this->nname_ = ros::this_node::getName();
}

TidInterface::~TidInterface(void) {
	this->Detach();
	delete tobiid_;
}

bool TidInterface::configure(void) {

	bool retcode = true;
	
	// Getting parameters
	this->p_nh_.getParam("loopip", this->loop_ip_);
	this->p_nh_.getParam("reconnect", this->rtime_);
	this->p_nh_.getParam("pipe", this->pipe_);
	this->p_nh_.getParam("mode", this->nmode_);

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

	return retcode;
}

bool TidInterface::Attach(void) {

	bool retcode = false;
	if(this->IsConnected())
		retcode = this->tobiid_->Attach(this->pipe_);
	
	return retcode;
}

bool TidInterface::Detach(void) {
	bool retcode = false;
	if(this->IsConnected())
		retcode = this->tobiid_->Detach();
	
	return retcode;
}

void TidInterface::on_received_ros2tid(const cnbiros_tobi_msgs::TidMessage& msg) {
	ROS_INFO("[%s] - Received TiD message on topic: %s", this->nname_.c_str(), this->stopic_.c_str());
}

bool TidInterface::Run(void) {
	
	ros::Rate r(20);

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

	while(this->nh_.ok()) {


		ros::spinOnce();
		r.sleep();
	}
}

/*
bool TidInterface::on_set_tid_(cnbiros_bci::SetTid::Request& req,
								cnbiros_bci::SetTid::Response& res) {

	ROS_INFO("Requested new connection to cnbi loop on pipe %s", req.pipe.c_str());
	res.result = this->Attach(req.pipe);

	return res.result;
}

bool TidInterface::on_unset_tid_(cnbiros_bci::UnSetTid::Request& req,
								  cnbiros_bci::UnSetTid::Response& res) {

	res.result = true;
	ROS_INFO("Requested to remove connection on pipe %s", req.pipe.c_str());
	
	if(this->tidclset_->Remove(req.pipe)) {
		ROS_INFO("Removed connection to cnbi loop on pipe %s", req.pipe.c_str());
	} else {
		ROS_WARN("Cannot remove connection on pipe %s, it does not exist", req.pipe.c_str());
		res.result = false;
	}
	
	return res.result;
}

void TidInterface::Run(void) {

	IDMessage 		  		cnbiIdm;
	IDSerializerRapid 		cnbiIds(&cnbiIdm);
	TidTools 		  		tidtool;
	cnbiros_tobi_msgs::TidMessage rosIdm;

	ros::Rate r(50);
	while(this->rosnode_->ok()) {

		for(auto it = this->tidclset_->Begin(); it != this->tidclset_->End(); ++it) {
			
			if(it->second->GetMessage(&cnbiIds) == true) {
				rosIdm = tidtool.GetMessage(cnbiIdm, it->first);
				this->pubset_->Publish(CNBIROS_BCI_TID_CNBI2ROS, rosIdm);
			}
		}
	
		r.sleep();
		ros::spinOnce();
	}
}

bool TidInterface::Detach(const std::string& pipe) {

	std::shared_ptr<ClTobiId> ptid = nullptr;
	bool result;
	if(this->tidclset_->Get(pipe, ptid))
		result = ptid->Detach();

	return result;
}


void TidInterface::callback_ros2tid(const cnbiros_tobi_msgs::TidMessage& rosIdm) {

	std::shared_ptr<ClTobiId>   ptid;
	bool 		retcod = false;

	IDMessage		  		cnbiIdm;
	IDSerializerRapid 		cnbiIds(&cnbiIdm);
	TidTools 				tidtool;

	// Convert ROS message to TidMessage
	cnbiIdm = tidtool.GetMessage(rosIdm);

	// Check if the corresponding ClTobiId exists and is attached
	if(this->tidclset_->Get(rosIdm.pipe, ptid)) {
		if(ptid->IsAttached()) {
			ptid->SetMessage(&cnbiIds);
			retcod = true;
		} 		
	}
	
	if(retcod == false) {
		ROS_ERROR("Message received from ROS to CNBILoop. " 
			      "However, no connection to %s has been found.", rosIdm.pipe.c_str());	
	}

}
*/
	}
}

#endif

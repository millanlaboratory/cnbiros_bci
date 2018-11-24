#ifndef CNBIROS_BCI_TIDINTERFACE_CPP
#define CNBIROS_BCI_TIDINTERFACE_CPP

#include "cnbiros_bci/TidInterface.hpp"

namespace cnbiros {
	namespace bci {

TidInterface::TidInterface(void) : p_nh_("~") {

	this->stopic_ = "rostid_ros2cnbi";
	this->ptopic_ = "rostid_cnbi2ros";


	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &TidInterface::on_received_ros2tid, this);
	this->pub_ = this->nh_.advertise<cnbiros_tobi_msgs::TidMessage>(this->ptopic_, 1);

	this->nname_ = ros::this_node::getName();
}

TidInterface::~TidInterface(void) {
	this->Detach();
	if(this->tobiid_ != nullptr)
		delete this->tobiid_;
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

	// Instantiate cltobiid with the correct mode
	this->tobiid_  = new ClTobiId(this->GetMode());

	return retcode;
}

bool TidInterface::Attach(void) {

	if(this->IsAttached())
		return true;
		
	if(this->tobiid_->Attach(this->pipe_)) {
		ROS_INFO("[%s] - Attached to %s pipe as %s", 
				 this->nname_.c_str(), this->pipe_.c_str(),
				 this->GetModeName().c_str());
	}

	return this->IsAttached();
}

bool TidInterface::Detach(void) {
	bool retcode = false;

	if(this->IsAttached() == false)
		return true;
		
	if(this->tobiid_->Detach())
		ROS_WARN("[%s] - Detached from %s pipe", this->nname_.c_str(), this->pipe_.c_str());
	
	return !(this->IsAttached());
}

bool TidInterface::IsAttached(void) {

	bool ret = false;
	if(this->tobiid_ != nullptr)
		ret = this->tobiid_->IsAttached();

	return ret;
}

void TidInterface::on_received_ros2tid(const cnbiros_tobi_msgs::TidMessage& msg) {
	ROS_DEBUG("[%s] - Received TiD message on topic: %s", this->nname_.c_str(), this->stopic_.c_str());
	this->fromRosMsg_ = msg;
	this->has_ros_message_ = true;
}

bool TidInterface::Run(void) {

	IDSerializerRapid				sloop(&(this->fromLoopMsg_));
	IDSerializerRapid				sros(&(this->toLoopMsg_));

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

		// Getting id message from loop
		if(this->IsAttached() && this->tobiid_->GetMessage(&sloop) == true) {
			ROS_DEBUG("[%s] - Received TiD message from loop", this->nname_.c_str());
			if(TidTools::ToRos(this->fromLoopMsg_, this->pipe_, this->toRosMsg_)) 
				this->pub_.publish(this->toRosMsg_);
		}

		// Getting id message from ros
		if(this->IsAttached() && this->has_ros_message_ == true) {
			if(TidTools::ToTobi(this->fromRosMsg_, this->toLoopMsg_))
				this->tobiid_->SetMessage(&sros);
			this->has_ros_message_ = false;
		}

		ros::spinOnce();
		r.sleep();
	}
}

	}
}

#endif

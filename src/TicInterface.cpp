#ifndef CNBIROS_BCI_TICINTERFACE_CPP
#define CNBIROS_BCI_TICINTERFACE_CPP

#include "cnbiros_bci/TicInterface.hpp"

namespace cnbiros {
	namespace bci {

TicInterface::TicInterface(void) : p_nh_("~") {

	this->stopic_ = "rostic_ros2cnbi";
	this->ptopic_ = "rostic_cnbi2ros";
	this->tobiic_ = nullptr;
	this->has_ros_message_ = false;

	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &TicInterface::on_received_ros2tic, this);
	this->pub_ = this->nh_.advertise<cnbiros_tobi_msgs::TicMessage>(this->ptopic_, 1);

	this->nname_ = ros::this_node::getName();
}

TicInterface::~TicInterface(void) {

	this->Detach();
	TicTools::Destroy(this->toLoopMsg_);
	this->destroy_tobiic();
}

void TicInterface::init_tobiic(void) {

	this->destroy_tobiic();

	if(this->IsAttached() == true)
		this->Detach();

	this->tobiic_ = new ClTobiIc(this->GetMode());
}

void TicInterface::destroy_tobiic(void) {
	if(this->tobiic_ != nullptr)
		delete this->tobiic_;
	this->tobiic_ = nullptr;
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
	this->init_tobiic();

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

bool TicInterface::ReAttach(void) {
	
	if(this->IsAttached() == true)
		return true;

	this->destroy_tobiic();
	this->init_tobiic();
	return this->Attach();
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
	
	if(TicTools::Empty(this->fromRosMsg_) == false)
		this->has_ros_message_ = true;
}

bool TicInterface::Run(void) {

	ICSerializerRapid				sloop(&(this->fromLoopMsg_));
	ICSerializerRapid				sros(&(this->toLoopMsg_));

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
		if(this->ReAttach() == false) {
			ROS_ERROR_THROTTLE(5.0f, "[%s] - Cannot attach to %s pipe", this->nname_.c_str(), this->pipe_.c_str());
		} 

		// Getting ic message from loop (GetOnly)
		if(this->GetMode() == TobiInterface::GetOnly) {
			switch(this->tobiic_->GetMessage(&sloop)) {
				case ClTobiIc::Detached:
					ROS_WARN("[%s] - Connection on pipe %s detached. Try to attach again..", 
							      this->nname_.c_str(), this->pipe_.c_str());
					break;
				case ClTobiIc::HasMessage:
					ROS_INFO_ONCE("[%s] - First message received from pipe %s", 
							      this->nname_.c_str(), this->pipe_.c_str());
					if(TicTools::ToRos(this->fromLoopMsg_, this->pipe_, this->toRosMsg_) == true)
						this->pub_.publish(this->toRosMsg_);
					break;
				case ClTobiIc::NoMessage:
					break;
			}
		// Getting ic message from ros (SetOnly)
		} else if(this->GetMode() == TobiInterface::SetOnly) {
			if(this->IsAttached() && this->has_ros_message_ == true) {
				ROS_INFO_ONCE("[%s] - Received TiC message from ros", this->nname_.c_str());

				if(TicTools::ToTobi(this->fromRosMsg_, this->toLoopMsg_) == true)
					this->tobiic_->SetMessage(&sros);
				
				this->has_ros_message_ = false;
			}

		}

		ros::spinOnce();
		r.sleep();
	}

	TicTools::Destroy(this->toLoopMsg_);
}

	}
}

#endif

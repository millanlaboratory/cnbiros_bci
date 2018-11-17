#ifndef CNBIROS_BCI_TOBIINTERFACE_CPP
#define CNBIROS_BCI_TOBIINTERFACE_CPP


#include "cnbiros_bci/TobiInterface.hpp"

namespace cnbiros {
	namespace bci {

TobiInterface::TobiInterface(void) {
	this->nmode_		 = "Undefined";
	this->mode_			 = TobiInterface::Undefined;
	this->loop_ip_		 = "127.0.0.1:8123";
	this->rtime_         = 0.0f;
}

TobiInterface::TobiInterface(const std::string& loopip, float rtime) {
	this->loop_ip_ = loopip;
	this->rtime_   = rtime;
}

TobiInterface::~TobiInterface(void) {
	this->Disconnect();
}

bool TobiInterface::Connect(void) {

	if(ClLoop::Configure(this->loop_ip_) == false) {
		ROS_ERROR("[%s] - Cannot configure the cnbi loop with this ip: %s", 
				  ros::this_node::getName().c_str(), this->loop_ip_.c_str());
		return false;
	}

	while(ClLoop::Connect() == false && this->rtime_ > 0.0f) {
		ROS_WARN("[%s] - Cannot establish connection to cnbi loop (%s)."
				 "Retrying in %f second(s)...", 
				 ros::this_node::getName().c_str(),
			     this->loop_ip_.c_str(), this->rtime_);
	}

	if(this->IsConnected()) {
		ROS_INFO("[%s] - Connection to cnbi loop at %s", 
				 ros::this_node::getName().c_str(), this->loop_ip_.c_str());
	} else {
		ROS_ERROR("[%s] - Cannot connect to cnbi loop at %s", 
				  ros::this_node::getName().c_str(), this->loop_ip_.c_str());
	}

	return this->IsConnected();
}

void TobiInterface::Disconnect(void) {
	if(ClLoop::IsConnected() == true)
		ClLoop::Disconnect();
}

bool TobiInterface::IsConnected(void) {
	return ClLoop::IsConnected();
}

bool TobiInterface::SetMode(const std::string& nmode) {

	bool retcode;

	if(!strcmp(nmode_.c_str(), "SetOnly")) {
		retcode = this->SetMode(TobiInterface::SetOnly);
	} else if(!(strcmp(this->nmode_.c_str(), "GetOnly"))) {
		retcode = this->SetMode(TobiInterface::GetOnly);
	} else if(!(strcmp(this->nmode_.c_str(), "SetGet"))) {
		retcode = this->SetMode(TobiInterface::SetGet);
	} else {
		retcode = false;
	}

	return retcode;
}

bool TobiInterface::SetMode(int mode) {
	
	bool retcode = true;

	switch(mode) {
		case TobiInterface::SetOnly:
		case TobiInterface::GetOnly:
		case TobiInterface::SetGet:
			this->mode_ = mode;
			break;
		default:
			this->mode_ = TobiInterface::Undefined;
			retcode = false;
			break;
	}
	return retcode;
}

int TobiInterface::GetMode(void) {
	return this->mode_;
}

std::string TobiInterface::GetModeName(void) {
	return this->nmode_;
}


	}
}


#endif

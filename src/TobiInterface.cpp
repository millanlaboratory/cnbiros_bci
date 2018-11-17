#ifndef CNBIROS_BCI_TOBIINTERFACE_CPP
#define CNBIROS_BCI_TOBIINTERFACE_CPP


#include "cnbiros_bci/TobiInterface.hpp"

namespace cnbiros {
	namespace bci {

TobiInterface::TobiInterface(void) {
	this->is_configured_ = false;
	this->is_attached_   = false;
	this->pipe_			 = "";
	this->nmode_		 = "Undefined";
	this->mode_			 = TobiInterface::Undefined;

	if(this->configure() == false)
		throw std::runtime_error("Configuration failed.");

}

TobiInterface::~TobiInterface(void) {
	this->Disconnect();
}

bool TobiInterface::configure(void) {

	bool retcode = true;

	// Getting parameters
	this->p_nh_.param<std::string>("loopip", this->loop_ip_, "127.0.0.1:8123");
	this->p_nh_.param<float>("reconnect", this->rtime_, 5.0f);
	this->p_nh_.param<std::string>("pipe", this->pipe_);
	this->p_nh_.param<std::string>("mode", this->nmode_);

	// Set interface mode
	if(this->SetMode(this->nmode_) == false) {
		ROS_ERROR("Unknown interface mode provided (%s). Aborting.", this->nmode_.c_str());
		retcode = false;
	}

	// Check if the pipe is provided
	if(this->pipe_.empty() == true) {
		ROS_ERROR("No cnbi pipe is provided. Aborting");
		retcode = false;
	}

	// Get reconnect option
	this->do_reconnect_ = false;
	if(this->rtime_ > 0.0f)
		this->do_reconnect_ = true;

	return retcode;
}

bool TobiInterface::Connect(void) {

	if(this->IsConfigured() == false) {
		ClLoop::Configure(this->loop_ip_);
		this->is_configured_ = true;
	}

	if(this->IsConnected() == true)
		return true;

	while(ClLoop::Connect() == false && this->do_reconnect_ == true) {
		ROS_WARN("Cannot establish connection to cnbi loop (%s)."
				 "Retrying in %f second(s)...", 
			     this->loop_ip_.c_str(), this->rtime_);
	}

	if(this->IsConnected())
		ROS_INFO("Connection to cnbi loop at %s", this->loop_ip_.c_str());
	else
		ROS_ERROR("Cannot connect to cnbi loop at %s", this->loop_ip_.c_str());
	

	return this->IsConnected();
}

void TobiInterface::Disconnect(void) {
	if(ClLoop::IsConnected() == true)
		ClLoop::Disconnect();
}

bool TobiInterface::IsConnected(void) {
	return this->IsConfigured() && ClLoop::IsConnected();
}

bool TobiInterface::IsConfigured(void) {
	return this->is_configured_;
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

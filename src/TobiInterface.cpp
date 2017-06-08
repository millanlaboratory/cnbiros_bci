#ifndef CNBIROS_BCI_TOBIINTERFACE_CPP
#define CNBIROS_BCI_TOBIINTERFACE_CPP


#include "cnbiros_bci/TobiInterface.hpp"

namespace cnbiros {
	namespace bci {

TobiInterface::TobiInterface(CcAddress address) {
	this->address_ = address;
	ClLoop::Configure(this->address_);
}

TobiInterface::~TobiInterface(void) {
	this->Disconnect();
}

void TobiInterface::Connect(void) {
	
	ros::Rate r(1);
	do {
		if(ClLoop::Connect() == false)
			ROS_ERROR_ONCE("Cannot establish connection to CNBI Loop (%s)."
					       "Retrying...", this->address_.c_str());
		r.sleep();
	} while(ClLoop::IsConnected() == false);
	
	ROS_INFO("Connection to CNBI Loop (%s)", this->address_.c_str());
}

void TobiInterface::Disconnect(void) {
	if(ClLoop::IsConnected() == true) {
		ClLoop::Disconnect();
	}
}

bool TobiInterface::IsConnected(void) {
	return ClLoop::IsConnected();
}

std::string TobiInterface::GetAddress(void) {
	return this->address_;
}


	}
}


#endif

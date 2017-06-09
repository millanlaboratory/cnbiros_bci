#ifndef CNBIROS_BCI_TICINTERFACE_CPP
#define CNBIROS_BCI_TICINTERFACE_CPP

#include "cnbiros_bci/TicInterface.hpp"

namespace cnbiros {
	namespace bci {

TicInterface::TicInterface(ros::NodeHandle* node, CcAddress address) : TobiInterface(address) {
	this->rosnode_  = node;
	this->pubset_   = new core::SetPublishers(node);
	this->subset_   = new core::SetSubscribers(node);
	this->ticclset_ = new TicClientSet;

	// Create and advertise the ROS topic
	this->pubset_->Add<cnbiros_bci::TicMessage>(CNBIROS_BCI_TIC_CNBI2ROS);
	
	// Create and subscribe to ROS topic
	this->subset_->Add(CNBIROS_BCI_TIC_ROS2CNBI, &TicInterface::callback_ros2tic, this);
};

TicInterface::~TicInterface(void) {
	delete this->pubset_;
	delete this->subset_;
	delete this->ticclset_;
}

bool TicInterface::Attach(const std::string& pipe, unsigned int mode) {

	ClTobiIc* ptic = nullptr;
	bool retcod    = false;

	// Remove ClTobiIc (if already exists)
	this->ticclset_->Remove(pipe);

	// Create ClTobiIc as GetOnly or SetOnly according to the provided mode
	switch(mode) {
		case TicInterface::ToRos:
			this->ticclset_->Add(pipe, ClTobiIc::GetOnly);
			break;
		case TicInterface::ToCnbi:
			this->ticclset_->Add(pipe, ClTobiIc::SetOnly);
	}

	// Retrieve pointer to the ClTobiIc added and try to attach
	if(this->ticclset_->Get(pipe, ptic))
		retcod = ptic->Attach(pipe);

	if (retcod == true)
		ROS_INFO("Attached to the CNBILoop at: %s", pipe.c_str());

	return retcod;
}


void TicInterface::Run(void) {

	ICMessage 		  		cnbiIcm;
	ICSerializerRapid 		cnbiIcs(&cnbiIcm);
	TicTools 		  		tictool;
	std::vector<cnbiros_bci::TicMessage> rosIcmList;

	ros::Rate r(50);
	while(this->rosnode_->ok()) {

		for(auto it = this->ticclset_->Begin(); it != this->ticclset_->End(); ++it) {
	
			if(it->second->GetMode() == ClTobiIc::GetOnly) {
				
				if(it->second->IsAttached()) {
					if(it->second->GetMessage(&cnbiIcs) == ClTobiIc::HasMessage) {
						ROS_INFO_ONCE("First message received from %s", it->first.c_str());
						rosIcmList = tictool.GetMessage(cnbiIcm, it->first);
						for(auto itm = rosIcmList.begin(); itm != rosIcmList.end(); ++itm) {
							this->pubset_->Publish(CNBIROS_BCI_TIC_CNBI2ROS, (*itm));
						}

					}
				} else {
					this->Attach(it->first, ClTobiIc::SetOnly);
				}
			}
		}
	
		r.sleep();
		ros::spinOnce();
	}
}

void TicInterface::Detach(const std::string& pipe) {

	ClTobiIc* ptic = nullptr;
	if(this->ticclset_->Get(pipe, ptic))
		ptic->Detach();
}


void TicInterface::callback_ros2tic(const cnbiros_bci::TicMessage& rosIcm) {

	ClTobiIc*   ptic;
	bool 		retcod = false;

	ICMessage		  		cnbiIcm;
	ICSerializerRapid 	cnbiIcs(&cnbiIcm);
	TicTools 				tictool;

	// Convert ROS message to TicMessage
	cnbiIcm = tictool.GetMessage(rosIcm);

	// Check if the corresponding ClTobiIc exists and is attached
	if(this->ticclset_->Get(rosIcm.pipe, ptic)) {
		if(ptic->IsAttached()) {
			ROS_INFO_ONCE("First message streamed from ROS to %s", rosIcm.pipe.c_str());
			ptic->SetMessage(&cnbiIcs);
			retcod = true;
		} else {
			ptic->Attach(rosIcm.pipe);
		}
	}
	
	if(retcod == false) {
		ROS_ERROR_THROTTLE(10, "Message received from ROS to CNBILoop. " 
			      "However, no connection to %s has been found.", rosIcm.pipe.c_str());	
	}

}

	}
}

#endif

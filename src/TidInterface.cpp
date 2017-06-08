#ifndef CNBIROS_BCI_TIDINTERFACE_CPP
#define CNBIROS_BCI_TIDINTERFACE_CPP

#include "cnbiros_bci/TidInterface.hpp"

namespace cnbiros {
	namespace bci {

TidInterface::TidInterface(ros::NodeHandle* node, CcAddress address) : TobiInterface(address) {
	this->rosnode_  = node;
	this->pubset_   = new core::SetPublishers(node);
	this->subset_   = new core::SetSubscribers(node);
	this->tidclset_ = new TidClientSet;
};

TidInterface::~TidInterface(void) {
	delete this->pubset_;
	delete this->subset_;
	delete this->tidclset_;
}

bool TidInterface::Attach(const std::string& pipe) {

	ClTobiId* ptid = nullptr;
	bool retcod    = false;

	// Remove ClTobiId (if already exists)
	this->tidclset_->Remove(pipe);
	
	// Add new ClTobiId
	this->tidclset_->Add(pipe, ClTobiId::SetGet);

	// Create and advertise the ROS topic
	this->pubset_->Add<cnbiros_bci::TidMessage>(CNBIROS_BCI_CNBI2ROS);
	
	// Create and subscribe to ROS topic
	this->subset_->Add(CNBIROS_BCI_ROS2CNBI, &TidInterface::callback_ros2tid, this);
	
	// Retrieve pointer to the ClTobiId added and try to attach
	if(this->tidclset_->Get(pipe, ptid))
		retcod = ptid->Attach(pipe);

	if (retcod == true)
		ROS_INFO("Attached to the CNBILoop at: %s", pipe.c_str());

	return retcod;
}


void TidInterface::Run(void) {

	IDMessage 		  		cnbiIdm;
	IDSerializerRapid 		cnbiIds(&cnbiIdm);
	TidTools 		  		tidtool;
	cnbiros_bci::TidMessage rosIdm;

	ros::Rate r(50);
	while(this->rosnode_->ok()) {

		for(auto it = this->tidclset_->Begin(); it != this->tidclset_->End(); ++it) {
			
			if(it->second->GetMessage(&cnbiIds) == true) {
				rosIdm = tidtool.GetMessage(cnbiIdm, it->first);
				this->pubset_->Publish(CNBIROS_BCI_CNBI2ROS, rosIdm);
			}
		}
	
		r.sleep();
		ros::spinOnce();
	}
}

void TidInterface::Detach(const std::string& pipe) {

	ClTobiId* ptid = nullptr;
	if(this->tidclset_->Get(pipe, ptid))
		ptid->Detach();
}


void TidInterface::callback_ros2tid(const cnbiros_bci::TidMessage& rosIdm) {

	ClTobiId*   ptid;
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

	}
}

#endif

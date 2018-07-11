#ifndef CNBIROS_BCI_TIDINTERFACE_CPP
#define CNBIROS_BCI_TIDINTERFACE_CPP

#include "cnbiros_bci/TidInterface.hpp"

namespace cnbiros {
	namespace bci {

TidInterface::TidInterface(ros::NodeHandle* node, CcAddress address) : TobiInterface(address) {
	this->rosnode_  = node;
	this->pubset_   = new core::Publishers(node);
	this->subset_   = new core::Subscribers(node);
	this->tidclset_ = new TidClientSet;

	// Create and advertise the ROS topic
	this->pubset_->Add<cnbiros_tobi_msgs::TidMessage>(CNBIROS_BCI_TID_CNBI2ROS);
	
	// Create and subscribe to ROS topic
	this->subset_->Add(CNBIROS_BCI_TID_ROS2CNBI, &TidInterface::callback_ros2tid, this);
	
	// Add services
	this->rossrv_set_tid_ = node->advertiseService(
					  	     ros::this_node::getName() + "/set_tid", &TidInterface::on_set_tid_, this);
	
	this->rossrv_unset_tid_ = node->advertiseService(
					  	      ros::this_node::getName() + "/unset_tid", &TidInterface::on_unset_tid_, this);
}

TidInterface::~TidInterface(void) {
	delete this->pubset_;
	delete this->subset_;
	delete this->tidclset_;
}

bool TidInterface::Attach(const std::string& pipe) {

	std::shared_ptr<ClTobiId> ptid = nullptr;
	bool retcod    = false;

	// Remove ClTobiId (if already exists)
	if(this->tidclset_->Remove(pipe)) {
		ROS_WARN("Removed previous connection to cnbi loop on pipe %s", pipe.c_str());
	}
	
	// Add new ClTobiId
	retcod = this->tidclset_->Add(pipe, ClTobiId::SetGet);

	if(retcod == false) {
		ROS_ERROR("Cannot add a new connection to cnbi loop on pipe %s: already exists", pipe.c_str());
		return false;
	} else {
		ROS_INFO("Added new connection to cnbi loop on pipe %s", pipe.c_str());
	}
	
	// Retrieve pointer to the ClTobiId added and try to attach
	if(this->tidclset_->Get(pipe, ptid))
		retcod = ptid->Attach(pipe);

	if (retcod == true)
		ROS_INFO("Connection to cnbi loop attached on pipe: %s", pipe.c_str());

	return retcod;
}

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

	}
}

#endif

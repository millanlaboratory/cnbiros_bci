#ifndef CNBIROS_BCI_TICINTERFACE_CPP
#define CNBIROS_BCI_TICINTERFACE_CPP

#include "cnbiros_bci/TicInterface.hpp"

namespace cnbiros {
	namespace bci {

TicInterface::TicInterface(ros::NodeHandle* node, CcAddress address) : TobiInterface(address) {
	this->rosnode_  = node;
	this->pubset_   = new core::Publishers(node);
	this->subset_   = new core::Subscribers(node);
	this->ticclset_ = new TicClientSet;

	// Create and advertise the ROS topic
	this->pubset_->Add<cnbiros_bci::TicMessage>(CNBIROS_BCI_TIC_CNBI2ROS);
	
	// Create and subscribe to ROS topic
	this->subset_->Add(CNBIROS_BCI_TIC_ROS2CNBI, &TicInterface::callback_ros2tic, this);

	// Add services
	this->rossrv_set_tic_ = node->advertiseService(
					  	     ros::this_node::getName() + "/set_tic", &TicInterface::on_set_tic, this);
	
	this->rossrv_unset_tic_ = node->advertiseService(
					  	      ros::this_node::getName() + "/unset_tic", &TicInterface::on_unset_tic, this);
	
	this->rossrv_sync_ = node->advertiseService(
					  	     ros::this_node::getName() + "/sync", &TicInterface::on_sync, this);

	// Initialize serializers
	this->cnbiicm_ = new ICMessage;
	this->cnbiics_ = new ICSerializerRapid(this->cnbiicm_);

	this->syncidx_ = TCBlock::BlockIdxUnset;
};

TicInterface::~TicInterface(void) {
	delete this->pubset_;
	delete this->subset_;
	delete this->ticclset_;
}

bool TicInterface::Attach(const std::string& pipe, unsigned int mode) {

	std::shared_ptr<ClTobiIc> ptic = nullptr;
	std::string lmode;
	bool retcod    = false;

	// Remove ClTobiIc (if already exists)
	if(this->ticclset_->Remove(pipe)) {
		ROS_WARN("Removed previous connection to cnbi loop on pipe %s", pipe.c_str());
	}
	
	// Create ClTobiIc as GetOnly or SetOnly according to the provided mode
	switch(mode) {
		case TicInterface::ToRos:
			lmode  = "cnbi2ros";
			retcod = this->ticclset_->Add(pipe, ClTobiIc::GetOnly);
			break;
		case TicInterface::ToCnbi:
			lmode  = "ros2cnbi";
			retcod = this->ticclset_->Add(pipe, ClTobiIc::SetOnly);
	}

	if(retcod == false) {
		ROS_ERROR("Cannot add a new connection to cnbi loop on pipe %s: already exists", pipe.c_str());
		return false;
	} else {
		ROS_INFO("Added new connection to cnbi loop (%s) on pipe %s", lmode.c_str(), pipe.c_str());
	}

	// Retrieve pointer to the ClTobiIc added and try to attach
	if(this->ticclset_->Get(pipe, ptic))
		retcod = ptic->Attach(pipe);

	if (retcod == true)
		ROS_INFO("Connection to cnbi loop attached on pipe: %s", pipe.c_str());

	return retcod;
}

bool TicInterface::on_set_tic(cnbiros_bci::SetTic::Request& req,
							  cnbiros_bci::SetTic::Response& res) {

	std::string lmode;

	switch(req.mode) {
		case TicInterface::ToRos:
			lmode = "cnbi2ros";
			break;
		case TicInterface::ToCnbi:
			lmode = "ros2cnbi";
			break;
	}

	ROS_INFO("Requested new connection to cnbi loop (%s) on pipe %s", lmode.c_str(), req.pipe.c_str());
	res.result = this->Attach(req.pipe, req.mode);

	return res.result;
}

bool TicInterface::on_unset_tic(cnbiros_bci::UnSetTic::Request& req,
								cnbiros_bci::UnSetTic::Response& res) {

	res.result = true;
	ROS_INFO("Requested to remove connection on pipe %s", req.pipe.c_str());
	
	if(this->ticclset_->Remove(req.pipe)) {
		ROS_INFO("Removed connection to cnbi loop on pipe %s", req.pipe.c_str());
	} else {
		ROS_WARN("Cannot remove connection on pipe %s, it does not exist", req.pipe.c_str());
		res.result = false;
	}
	
	return res.result;
}

bool TicInterface::on_sync(cnbiros_bci::Sync::Request &req,
							   cnbiros_bci::Sync::Response &res) {

	this->syncidx_ = req.syncidx;
	ROS_INFO("Sync tic pipes at NDF=%d", this->syncidx_);
	res.result = true;
	return res.result;
}

void TicInterface::Run(void) {

	//ICMessage 		  		cnbiIcm;
	//ICSerializerRapid 		cnbiIcs(&cnbiIcm);
	TicTools 		  		tictool;
	std::vector<cnbiros_bci::TicMessage> rosIcmList;
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

bool TicInterface::Detach(const std::string& pipe) {

	std::shared_ptr<ClTobiIc> ptic = nullptr;
	bool result = false;
	if(this->ticclset_->Get(pipe, ptic)) {
		result = ptic->Detach();
	}

	return result;
	
}


void TicInterface::callback_ros2tic(const cnbiros_bci::TicMessage& rosIcm) {

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

	}
}

#endif

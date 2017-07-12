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
					  	     ros::this_node::getName() + "/set_tic", &TicInterface::on_set_tic_, this);
	
	this->rossrv_unset_tic_ = node->advertiseService(
					  	      ros::this_node::getName() + "/unset_tic", &TicInterface::on_unset_tic_, this);
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

bool TicInterface::on_set_tic_(cnbiros_bci::SetTic::Request& req,
								cnbiros_bci::SetTic::Response& res) {

	std::string ldir;

	switch(req.mode) {
		case TicInterface::ToRos:
			ldir = "cnbi2ros";
			break;
		case TicInterface::ToCnbi:
			ldir = "ros2cnbi";
			break;
	}

	ROS_INFO("Requested to attach to %s (%s)", req.pipe.c_str(), ldir.c_str());
	res.result = this->Attach(req.pipe, req.mode);

	return res.result;
}

bool TicInterface::on_unset_tic_(cnbiros_bci::UnSetTic::Request& req,
								 cnbiros_bci::UnSetTic::Response& res) {

	res.result = true;
	ROS_INFO("Requested to detach from %s", req.pipe.c_str());
	this->Detach(req.pipe);
	ROS_INFO("Detached from %s", req.pipe.c_str());

	this->ticclset_->Remove(req.pipe);
	
	return true;
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
				} 			
			}
		}
	
		r.sleep();
		ros::spinOnce();
	}
}

void TicInterface::Detach(const std::string& pipe) {

	ClTobiIc* ptic = nullptr;
	if(this->ticclset_->Get(pipe, ptic)) {
		if(ptic->IsAttached()) {
			ptic->Detach();
		}
	}
	
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

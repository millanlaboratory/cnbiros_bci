#ifndef CNBIROS_BCI_TIDTOOLS_CPP
#define CNBIROS_BCI_TIDTOOLS_CPP

#include "cnbiros_bci/TidTools.hpp"

namespace cnbiros {
	namespace bci {

TidTools::TidTools(void) {}

TidTools::~TidTools(void) {}

bool TidTools::ToTobi(const cnbiros_tobi_msgs::TidMessage& mros, IDMessage& mtobi) {

	bool ret = true;	
	// Fill the IDMessage with the ros message fields. Since ors has not a frame
	// block (as the cnbiloop), set the block id as unset.
	try {	
		mtobi.SetFamilyType(mros.family);
		mtobi.SetDescription(mros.description);
		mtobi.SetEvent(mros.event);
		mtobi.SetBlockIdx(TCBlock::BlockIdxUnset);
	} catch (TCException& e) {
		ROS_ERROR("[%s] - IDMessage conversion error: %s", ros::this_node::getName().c_str(), e.GetInfo().c_str());
		ret = false;
	}

	return ret;
}

bool TidTools::ToRos(const IDMessage& mtobi, const std::string& pipe, cnbiros_tobi_msgs::TidMessage& mros) {

	bool ret = true;
	
	try {
		mros.header.stamp	 = ros::Time::now();
		mros.header.frame_id = "base_link";
		mros.version  		 = "1.0.0";
		mros.family 		 = mtobi.GetFamilyType();
		mros.description 	 = mtobi.GetDescription();
		mros.event 			 = mtobi.GetEvent();
		mros.pipe  			 = pipe;
	} catch (std::runtime_error& e) {
		ROS_ERROR("[%s] - ID ros message conversion erros: %s", ros::this_node::getName().c_str(), e.what());
		ret = false;
	}

	return ret;
	
}

	}
}

#endif

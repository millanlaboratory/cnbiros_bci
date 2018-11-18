#ifndef CNBIROS_BCI_TIDTOOLS_CPP
#define CNBIROS_BCI_TIDTOOLS_CPP

#include "cnbiros_bci/TidTools.hpp"

namespace cnbiros {
	namespace bci {

TidTools::TidTools(void) {}

TidTools::~TidTools(void) {}

IDMessage TidTools::ToTobi(const cnbiros_tobi_msgs::TidMessage& idmros) {

	IDMessage idmcnbi;

	// Fill the IDMessage with the ros message fields. Since ors has not a frame
	// block (as the cnbiloop), set the block id as unset.
	idmcnbi.SetFamilyType(idmros.family);
	idmcnbi.SetDescription(idmros.description);
	idmcnbi.SetEvent(idmros.event);
	idmcnbi.SetBlockIdx(TCBlock::BlockIdxUnset);

	return idmcnbi;
}

cnbiros_tobi_msgs::TidMessage TidTools::ToRos(const IDMessage& idmcnbi, const std::string& pipe) {

	cnbiros_tobi_msgs::TidMessage idmros;

	idmros.header.stamp 	= ros::Time::now();
	idmros.header.frame_id  = "base_link";
	idmros.version  		= "1.0.0";
	idmros.family 			= idmcnbi.GetFamilyType();
	idmros.description 		= idmcnbi.GetDescription();
	idmros.event 			= idmcnbi.GetEvent();
	idmros.pipe  			= pipe;

	return idmros;
	
}

	}
}

#endif

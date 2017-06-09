#ifndef CNBIROS_BCI_TICTOOLS_CPP
#define CNBIROS_BCI_TICTOOLS_CPP

#include "cnbiros_bci/TicTools.hpp"

namespace cnbiros {
	namespace bci {

TicTools::TicTools(void) {}

TicTools::~TicTools(void) {}

ICMessage TicTools::GetMessage(const cnbiros_bci::TicMessage& icmros) {

	ICMessage 		icmcnbi;

	this->icclassifier_ = new ICClassifier(icmros.classifier.name,
							   				icmros.classifier.description,
							   				icmros.classifier.vtype,
							   				icmros.classifier.ltype);

	// Retrieving the classes from ROS message (and store them in the internal
	// class vector
	for(auto it = icmros.classifier.classes.begin(); it != icmros.classifier.classes.end(); ++it)
		this->icclasses_.push_back(new ICClass((*it).label, (*it).value));

	// Adding reference to the classes in the internal classifier 
	for (auto it = this->icclasses_.begin(); it != this->icclasses_.end(); ++it) {
		this->icclassifier_->classes.Add((*it));
	}

	// Adding reference to the classifier in the return ICMessage
	icmcnbi.classifiers.Add(this->icclassifier_);

	return icmcnbi;
}

std::vector<cnbiros_bci::TicMessage> TicTools::GetMessage(const ICMessage& icmcnbi, const std::string& pipe) {

	std::vector<cnbiros_bci::TicMessage> rosmessages;

	for(auto itcc = icmcnbi.classifiers.Begin(); itcc != icmcnbi.classifiers.End(); ++itcc) {

		cnbiros_bci::TicClassifier rosclassifier;

		rosclassifier.name 	   	   = itcc->second->GetName();
		rosclassifier.description  = itcc->second->GetDescription();
		rosclassifier.vtype 	   = itcc->second->GetValueType();
		rosclassifier.ltype 	   = itcc->second->GetLabelType();

		std::vector<cnbiros_bci::TicClass> rosclasses;
		for(auto itcl = itcc->second->classes.Begin(); itcl != itcc->second->classes.End(); ++itcl) {
			
			cnbiros_bci::TicClass rosclass;
			rosclass.label = itcl->second->GetLabel();
			rosclass.value = itcl->second->GetValue();
			rosclasses.push_back(rosclass);
		}
		rosclassifier.classes = rosclasses;

		
		cnbiros_bci::TicMessage icmros;
		icmros.header.stamp 	= ros::Time::now();
		icmros.header.frame_id 	= "base_link";
		icmros.version 			= "0.0.1";
		icmros.frame   			= icmcnbi.GetBlockIdx();
		icmros.pipe    			= pipe;
		icmros.classifier 		= rosclassifier;

		rosmessages.push_back(icmros);
	}

	return rosmessages;
	
}

	}
}

#endif

#ifndef CNBIROS_BCI_TICTOOLS_CPP
#define CNBIROS_BCI_TICTOOLS_CPP

#include "cnbiros_bci/TicTools.hpp"

namespace cnbiros {
	namespace bci {

TicTools::TicTools(void) {}

TicTools::~TicTools(void) {}

bool TicTools::ToTobi(const cnbiros_tobi_msgs::TicMessage& mros, ICMessage& mtobi) {

	ICClassifier* icc;
	ICClass*	  icl;
	bool ret = true;

	try {
		for(auto itcs = mros.classifiers.begin(); itcs != mros.classifiers.end(); ++itcs) {

			if(mtobi.classifiers.Has(itcs->name) == false) {
				icc = new ICClassifier(itcs->name, itcs->description, itcs->vtype, itcs->ltype);
				icc = mtobi.classifiers.Add(icc);
			} else {
				icc = mtobi.classifiers.Get(itcs->name);
			}

			for(auto itcl = itcs->classes.begin(); itcl != itcs->classes.end(); ++itcl) {
				
				if(icc->classes.Has(itcl->label) == false) {
					icl = new ICClass(itcl->label);
					icl = icc->classes.Add(icl);
				} else {
					icl = icc->classes.Get(itcl->label);
				}

				icl = icl->SetValue(itcl->value);
			}
		}
	} catch (TCException& e) {
		ROS_ERROR("[%s] - ICMessage conversion error: %s", ros::this_node::getName().c_str(), e.GetInfo().c_str());
		ret = false;
	}

	if(mtobi.classifiers.Empty() == true)
		ret = false;
	
	return ret;
}

cnbiros_tobi_msgs::TicMessage TicTools::ToRos(const ICMessage& mtobi, const std::string& pipe) {

	cnbiros_tobi_msgs::TicMessage mros;

	mros.header.stamp		= ros::Time::now();
	mros.header.frame_id 	= "base_link";
	mros.version 			= "1.0.0";
	mros.frame   			= mtobi.GetBlockIdx();
	mros.pipe    			= pipe;
	
	std::vector<cnbiros_tobi_msgs::TicClassifier> roscs_v;
	for (auto itcs = mtobi.classifiers.Begin(); itcs != mtobi.classifiers.End(); ++itcs) {
		
		cnbiros_tobi_msgs::TicClassifier roscs;

		roscs.name			= itcs->second->GetName();
		roscs.description	= itcs->second->GetDescription();
		roscs.vtype			= itcs->second->GetValueType();
		roscs.ltype			= itcs->second->GetLabelType();

		std::vector<cnbiros_tobi_msgs::TicClass> roscl_v;
		for(auto itcl = itcs->second->classes.Begin(); itcl != itcs->second->classes.End(); ++itcl) {
			cnbiros_tobi_msgs::TicClass roscl;
			roscl.label = itcl->second->GetLabel();
			roscl.value = itcl->second->GetValue();
			roscl_v.push_back(roscl);
		}
		roscs.classes = roscl_v;

		roscs_v.push_back(roscs);
	}

	mros.classifiers = roscs_v;

	return mros;
}

void TicTools::Destroy(ICMessage& msg) {

	if(msg.classifiers.Empty() == false) {
		for(auto itcs = msg.classifiers.Begin(); itcs != msg.classifiers.End(); ++itcs) {
			if(itcs->second->classes.Empty() == false)
				itcs->second->classes.Destroy();
		}
		msg.classifiers.Destroy();
	}
}

/*
ICMessage TicTools::GetMessage(const cnbiros_tobi_msgs::TicMessage& icmros) {

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

std::vector<cnbiros_tobi_msgs::TicMessage> TicTools::GetMessage(const ICMessage& icmcnbi, const std::string& pipe) {

	std::vector<cnbiros_tobi_msgs::TicMessage> rosmessages;

	for(auto itcc = icmcnbi.classifiers.Begin(); itcc != icmcnbi.classifiers.End(); ++itcc) {

		cnbiros_tobi_msgs::TicClassifier rosclassifier;

		rosclassifier.name 	   	   = itcc->second->GetName();
		rosclassifier.description  = itcc->second->GetDescription();
		rosclassifier.vtype 	   = itcc->second->GetValueType();
		rosclassifier.ltype 	   = itcc->second->GetLabelType();

		std::vector<cnbiros_tobi_msgs::TicClass> rosclasses;
		for(auto itcl = itcc->second->classes.Begin(); itcl != itcc->second->classes.End(); ++itcl) {
			
			cnbiros_tobi_msgs::TicClass rosclass;
			rosclass.label = itcl->second->GetLabel();
			rosclass.value = itcl->second->GetValue();
			rosclasses.push_back(rosclass);
		}
		rosclassifier.classes = rosclasses;

		
		cnbiros_tobi_msgs::TicMessage icmros;
		icmros.header.stamp 	= ros::Time::now();
		icmros.header.frame_id 	= "base_link";
		icmros.version 			= "1.0.0";
		icmros.frame   			= icmcnbi.GetBlockIdx();
		icmros.pipe    			= pipe;
		icmros.classifier 		= rosclassifier;

		rosmessages.push_back(icmros);
	}

	return rosmessages;
	
}

float TicTools::GetValue(const cnbiros_tobi_msgs::TicMessage& msg, const std::string& name, const std::string& label) {

	float value;
	bool found = false;
	if(msg.classifier.name.compare(name) != 0) {
		// raise exception
		ROS_ERROR("classifier not found");
	} else {
		for(auto itc = msg.classifier.classes.begin(); itc != msg.classifier.classes.end(); ++itc) {
			if((*itc).label.compare(label) == 0) {
				found = true;
				value = (*itc).value;
			}
		}
	}

	if(found == false)
		ROS_ERROR("class not found");

	return value;
}
*/
	}
}

#endif

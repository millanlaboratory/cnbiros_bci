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

bool TicTools::ToRos(const ICMessage& mtobi, const std::string& pipe, cnbiros_tobi_msgs::TicMessage& mros) {

	bool ret = true;
	mros.classifiers.clear();

	mros.header.stamp		= ros::Time::now();
	mros.header.frame_id 	= "base_link";
	mros.version 			= "1.0.0";
	mros.frame   			= mtobi.GetBlockIdx();
	mros.pipe    			= pipe;

	try {
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
	} catch (std::runtime_error& e) {
		ROS_ERROR("[%s] - IC ros message conversion erros: %s", ros::this_node::getName().c_str(), e.what());
		ret = false;
	}

	return ret;
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

bool TicTools::Empty(ICMessage& msg) {
	return msg.classifiers.Empty();
}

bool TicTools::Empty(cnbiros_tobi_msgs::TicMessage& msg) {
	return msg.classifiers.empty();
}

	}
}

#endif

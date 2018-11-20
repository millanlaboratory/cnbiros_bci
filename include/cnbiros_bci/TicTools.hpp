#ifndef CNBIROS_BCI_TICTOOLS_HPP
#define CNBIROS_BCI_TICTOOLS_HPP

#include <vector>
#include <ros/ros.h>
#include <tobicore/TCException.hpp>
#include <tobiic/ICMessage.hpp>
#include "cnbiros_tobi_msgs/TicClass.h"
#include "cnbiros_tobi_msgs/TicClassifier.h"
#include "cnbiros_tobi_msgs/TicMessage.h"

namespace cnbiros {
	namespace bci {

class TicTools {
	
	public:
		TicTools(void);
		~TicTools(void);

		//ICMessage GetMessage(const cnbiros_tobi_msgs::TicMessage& icmros);
		
		//std::vector<cnbiros_tobi_msgs::TicMessage> GetMessage(const ICMessage& iccnbi, const std::string& pipe);


		static bool ToTobi(const cnbiros_tobi_msgs::TicMessage& mros, ICMessage& mtobi);
		static cnbiros_tobi_msgs::TicMessage ToRos(const ICMessage& mtobi, const std::string& pipe);

		//float GetValue(const cnbiros_tobi_msgs::TicMessage& msg, const std::string& name, const std::string& label);

	private:
		std::vector<ICClass*> 	icclasses_;
		ICClassifier* 		 	icclassifier_;

};

	}
}

#endif

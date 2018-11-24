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

		static bool ToTobi(const cnbiros_tobi_msgs::TicMessage& mros, ICMessage& mtobi);
		static bool ToRos(const ICMessage& mtobi, const std::string& pipe, cnbiros_tobi_msgs::TicMessage& mros);

		static void Destroy(ICMessage& msg);
		static bool Empty(ICMessage& msg);
		static bool Empty(cnbiros_tobi_msgs::TicMessage& msg);


};

	}
}

#endif

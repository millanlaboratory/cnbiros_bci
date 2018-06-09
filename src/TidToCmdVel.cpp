#ifndef CNBIROS_BCI_TIDTOCMDVEL_CPP
#define CNBIROS_BCI_TIDTOCMDVEL_CPP

#include "cnbiros_bci/TidToCmdVel.hpp"
#include <xmlrpcpp/XmlRpc.h>
#include <XmlRpcValue.h>

namespace cnbiros {
	namespace bci {

TidToCmdVel::TidToCmdVel(void) : private_nh_("~") {

	// Configure node parameters
	if(this->configure() == false)
		ROS_ERROR("Configuration of TidToCmdVel failed");


	// Initialize subscriber and publisher
	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &TidToCmdVel::on_received_tid, this);
	this->pub_ = this->nh_.advertise<geometry_msgs::Twist>(this->ptopic_, 1);
}

TidToCmdVel::~TidToCmdVel(void) {}

bool TidToCmdVel::configure(void) {
		
	XmlRpc::XmlRpcValue dict;
	std::string	dict_key;
	geometry_msgs::Twist cmd;
	bool additive;

	// Getting parameters
	this->private_nh_.param<std::string>("source", this->stopic_, "/rostid_cnbi2ros");
	this->private_nh_.param<std::string>("target", this->ptopic_, "/cmd_vel");
	
	// Getting dictionary for the events<=>velocity association
	if(this->private_nh_.getParam("commands", dict) == false) {
		ROS_ERROR("Cannot retrieve required commands dictionary.");
		return false;
	}

	// Initialize twist message
	cmd.linear.x  = 0.0f;
	cmd.linear.y  = 0.0f;
	cmd.linear.z  = 0.0f;
	cmd.angular.x = 0.0f;
	cmd.angular.y = 0.0f;
	cmd.angular.z = 0.0f;

	try {
		for(auto i = 0; i<dict.size(); ++i) {
			if(dict[i].hasMember("event") == false || 
			   dict[i].hasMember("linear") == false ||
			   dict[i].hasMember("angular") == false ||
			   dict[i].hasMember("label") == false) {
				ROS_ERROR("Commands dictionary must have fields 'event', 'linear', 'angular' and 'label'");
				return false;
			}

			additive = false;
			if(dict[i].hasMember("additive") == true) {
				additive = static_cast<bool>(dict[i]["additive"]);
			}

			dict_key	  = static_cast<std::string>(dict[i]["event"]);
			cmd.linear.x  = static_cast<double>(dict[i]["linear"]);
			cmd.angular.z = this->deg2rad(static_cast<double>(dict[i]["angular"]));
			this->cmd_vel_[dict_key] = cmd;
			this->cmd_labels_[dict_key] = static_cast<std::string>(dict[i]["label"]);
			this->cmd_mode_[dict_key] = additive;
			
		}
	} catch (XmlRpc::XmlRpcException& ex) {
		ROS_ERROR("Wrong format for commands dictionary.");
		return false;
	}

	for(auto it=this->cmd_vel_.begin(); it!=this->cmd_vel_.end(); ++it) {
		auto itl = this->cmd_labels_.find(it->first.c_str());
		auto ita = this->cmd_mode_.find(it->first.c_str());
		ROS_INFO("Commands dictionary: ['%s'] => linear: %f, angular: %f, label: %s, additive: %d", it->first.c_str(), it->second.linear.x, this->rad2deg(it->second.angular.z), itl->second.c_str(), ita->second);
	}

	return true;
}

void TidToCmdVel::on_received_tid(const cnbiros_bci::TidMessage& msg) {

	geometry_msgs::Twist twist;
	std::stringstream sevent;
	
	sevent << std::showbase << std::hex << msg.event;
	
	auto itc = this->cmd_vel_.find(sevent.str());
	auto itl = this->cmd_labels_.find(sevent.str());
	auto ita = this->cmd_mode_.find(sevent.str());

	if(itc != this->cmd_vel_.end()) {
		ROS_INFO("Received TiD event: ['%s'] => linear: %f, angular: %f, label: %s", 
				  itc->first.c_str(), itc->second.linear.x, this->rad2deg(itc->second.angular.z), 
				  itl->second.c_str());
	
		if(ita->second == true) {
			this->current_cmd_.linear.x  += itc->second.linear.x;  
			this->current_cmd_.linear.y  += itc->second.linear.y;  
			this->current_cmd_.linear.z  += itc->second.linear.z;  
			this->current_cmd_.angular.x += itc->second.angular.x; 
			this->current_cmd_.angular.y += itc->second.angular.y; 
			this->current_cmd_.angular.z += itc->second.angular.z; 
		} else {
			this->current_cmd_ = itc->second;
		}

		this->pub_.publish(this->current_cmd_);
	} else {
	//	ROS_WARN("Unknown event: %s", itc->first.c_str());
	}

}

float TidToCmdVel::rad2deg(float rad) {
	return rad*180.0f/M_PI;
}

float TidToCmdVel::deg2rad(float deg) {
	return deg*M_PI/180.0f;
}

	}
}



#endif

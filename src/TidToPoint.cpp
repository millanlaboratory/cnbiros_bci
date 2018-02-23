#ifndef CNBIROS_BCI_TIDTOPOINT_CPP
#define CNBIROS_BCI_TIDTOPOINT_CPP

#include "cnbiros_bci/TidToPoint.hpp"
#include <xmlrpcpp/XmlRpc.h>
#include <XmlRpcValue.h>

namespace cnbiros {
	namespace bci {

TidToPoint::TidToPoint(void) : private_nh_("~") {

	// Configure node parameters
	if(this->configure() == false)
		ROS_ERROR("Configuration of TidToPoint failed");

	// Initialize subscriber and publisher
	this->sub_ = this->nh_.subscribe(this->stopic_, 50, &TidToPoint::on_received_tid, this);
	this->pub_ = this->nh_.advertise<geometry_msgs::PointStamped>(this->ptopic_, 50);
}

TidToPoint::~TidToPoint(void) {}

bool TidToPoint::configure(void) {


	XmlRpc::XmlRpcValue dict;
	std::string	dict_key;

	// Getting parameters
	this->private_nh_.param<std::string>("frame_id", this->frame_id_, "base_link");
	this->private_nh_.param<std::string>("source", this->stopic_, "/rostid_cnbi2ros");
	this->private_nh_.param<std::string>("point", this->ptopic_, "/point");
	this->private_nh_.param<float>("point", this->distance_, 1.0f);

	// Getting dictionary for the events<=>angles association
	if(this->private_nh_.getParam("commands", dict) == false) {
		ROS_ERROR("Cannot retrieve required commands dictionary.");
		return false;
	}

	try {
		for(auto i = 0; i<dict.size(); ++i) {
			if(dict[i].hasMember("event") == false || dict[i].hasMember("angle") == false) {
				ROS_ERROR("Commands dictionary must have fields 'event' and 'angle'");
				return false;
			}
			dict_key = static_cast<std::string>(dict[i]["event"]);
			this->commands_[dict_key] = static_cast<double>(dict[i]["angle"]);
		}
	} catch (XmlRpc::XmlRpcException& ex) {
		ROS_ERROR("Wrong format for commands dictionary.");
		return false;
	}

	for(auto it=this->commands_.begin(); it!=this->commands_.end(); ++it) {
		ROS_INFO("Configured commands: ['%s'] => %+6.2f [deg]", it->first.c_str(), it->second*180.0f/M_PI);
	}

	return true;
}

void TidToPoint::on_received_tid(const cnbiros_bci::TidMessage& msg) {

	geometry_msgs::PointStamped point;
	std::stringstream sevent;
	std::map<std::string, double>::iterator it;
	float angle;

	sevent << std::showbase << std::hex << msg.event;

	it = this->commands_.find(sevent.str());

	if(it != this->commands_.end()) {
		ROS_INFO("Received TiD event: ['%s'] => %+6.2f", it->first.c_str(), it->second*180.0f/M_PI);

		// Convert the angle from robot coordinate
		angle = it->second + M_PI/2.0f;
	
		// Create point message
		point.header.frame_id	= this->frame_id_;
		point.header.stamp		= ros::Time::now();
		point.point.x			= this->distance_*sin(angle);
		point.point.y			= -this->distance_*cos(angle);
		point.point.z			= 0.0f;

		// Publish the message
		this->pub_.publish(point);
	}
}


	}
}

#endif

#ifndef CNBIROS_BCI_TIDTOPOINT_HPP
#define CNBIROS_BCI_TIDTOPOINT_HPP

// System includes
#include <cmath>
#include <map>
#include <sstream>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

// Package includes
#include "cnbiros_bci/TidMessage.h"

namespace cnbiros {
	namespace bci {

/*!
 * TidToPoint class converts given TiD events in geometric points for further
 * use. It assumes that a received event represents the orientation with respect
 * to the heading direction of the robot. Given that and the radius, it computes
 * the cartesian position of a geometric point.
 *
 * The association between events<=>angles must be done in the yaml
 * configuration file.
 */
class TidToPoint {

	public:
		/*!
		 * Constructor.
		 */
		TidToPoint(void);

		/*!
		 * Destructor.
		 */
		virtual ~TidToPoint(void);

		/*!
		 * Configuration method. It reads from the ROS param server the
		 * configuration parameters for the node. In particular, it parse the
		 * dictionary for events<=>angles associations via XmlRpc.
		 */
		virtual bool configure(void);

	private:
		void on_received_tid(const cnbiros_bci::TidMessage& msg);

	private:
		ros::NodeHandle	nh_;
		ros::NodeHandle	private_nh_;

		ros::Subscriber	sub_;
		ros::Publisher  pub_;
		std::string		stopic_;
		std::string		ptopic_;

		std::string frame_id_;
		float distance_;
		std::map<std::string, double> commands_;
};

	}
}


#endif

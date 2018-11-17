#include <ros/ros.h>
#include <tobiid/IDMessage.hpp>
#include <tobiid/IDSerializerRapid.hpp>
#include <cnbiloop/ClLoop.hpp>
#include <cnbiloop/ClTobiId.hpp>
#include "cnbiros_bci/Sync.h"

ClTobiId 		  id(ClTobiId::SetOnly);
IDMessage		  idm;
IDSerializerRapid ids(&idm);
int 			  fidx = TCBlock::BlockIdxUnset;
bool 			  rossrv_exec = false;
cnbiros_bci::Sync rossrvm; 

void callback(const ros::TimerEvent& event) {
	ROS_INFO("Callback");
	rossrv_exec = true;
}

int main(int argc, char** argv) {
	
	// ros initialization
	ros::init(argc, argv, "rossync");
	ros::NodeHandle node;

	ros::ServiceClient rossrv = node.serviceClient<cnbiros_bci::Sync>("/rostic/sync");
	ros::Timer timer = node.createTimer(ros::Duration(1), callback);
	ros::Rate r(50);
	
	ClLoop::Configure();

	while(id.Attach("/dev") == false) {
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("rossync attached to cnbi /dev pipe");
	
	idm.SetEvent(1);
	while(node.ok()) {

		if(rossrv_exec == true) {
			id.SetMessage(&ids, TCBlock::BlockIdxUnset, &fidx);
			ros::Duration(0.08).sleep();
			id.SetMessage(&ids, TCBlock::BlockIdxUnset, &fidx);
			rossrvm.request.syncidx = fidx;
			if(rossrv.call(rossrvm)) {
				ROS_INFO("Sync requested");
			}

			rossrv_exec = false;
		}

		r.sleep();
		ros::spinOnce();
	}

	return 0;
}

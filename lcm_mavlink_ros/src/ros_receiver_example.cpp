#include "ros/ros.h"
#include "mavlink.h"
#include "lcm_mavlink_ros/Mavlink.h"
#include "mavlinkros.h"
#include <sstream>

bool verbose;

/**
 * Grabs all mavlink-messages from the ROS-Topic "mavlink" and prints them
 */

ros::Subscriber mavlink_sub;


void mavlinkCallback(const lcm_mavlink_ros::Mavlink::ConstPtr& mavlink_ros_msg)
{

	/**
	 * Convert lcm_mavlink_ros::Mavlink to mavlink_message_t
	 */
	mavlink_message_t msg;
	createMavlinkFromROS(mavlink_ros_msg,&msg);

	// We now have the message decoded and and can act on it
	
	switch (msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			ROS_INFO("HEARTBEAT from system %i",msg.sysid);
			break;
		default:
			break;
	}
	

	ROS_INFO("Received Mavlink from ROS, Message-ID: [%i]", mavlink_ros_msg->msgid);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rostolcm");
	ros::NodeHandle n;

	mavlink_sub = n.subscribe("mavlink", 1000, mavlinkCallback);


	/**
	 * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
	 */
	ros::spin();

	return 0;
}

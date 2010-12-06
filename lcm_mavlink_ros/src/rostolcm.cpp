#include "ros/ros.h"

#include "lcm_mavlink_ros/Mavlink.h"
#include "mavconn.h"
#include "mavlinkros.h"

#include <sstream>

/**
 * Grabs all mavlink-messages from the ROS-Topic "mavlink" and publishes them on the LCM-Mavlink-Channel
 */

ros::Subscriber mavlink_sub;
lcm_t *lcm;


void mavlinkCallback(const lcm_mavlink_ros::Mavlink::ConstPtr& mavlink_ros_msg)
{
	//Check if the message is coming from lcm so that it is not send back
	if (mavlink_ros_msg->fromlcm)
		return;

	/**
	 * Convert lcm_mavlink_ros::Mavlink to mavlink_message_t
	 */
	mavlink_message_t msg;
	createMavlinkFromROS(mavlink_ros_msg,&msg);

	/**
	 * Send mavlink_message to LCM (so that the rest of the MAVConn world can hear us)
	 */
	mavlink_message_t_publish (lcm, "MAVLINKROS", &msg);

	//DEBUG
	ROS_INFO("Sent Mavlink from ROS to LCM, Message-ID: [%i]", mavlink_ros_msg->msgid);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rostolcm");

	ros::NodeHandle n;

	mavlink_sub = n.subscribe("mavlink", 1000, mavlinkCallback);

	/**
	 * Connect to LCM Channel and register for MAVLink messages ->
	 */
	lcm = lcm_create("udpm://");
	if (!lcm) {
		return 1;
	}


	/**
	 * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
	 */
	ros::spin();

	lcm_destroy (lcm);

	return 0;
}

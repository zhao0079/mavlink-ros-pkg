#include "ros/ros.h"

#include "lcm_mavlink_ros/Mavlink.h"
#include "mavlinkros.h"
#include "mavlink.h"
#include <sstream>
#include <glib.h>

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

	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- receive and print MAVLink messages from ROS");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}

	ros::NodeHandle n;

	mavlink_sub = n.subscribe("mavlink", 1000, mavlinkCallback);


	/**
	 * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
	 */
	ros::spin();

	return 0;
}

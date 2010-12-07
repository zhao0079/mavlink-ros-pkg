#include "ros/ros.h"
#include "lcm_mavlink_ros/Mavlink.h"
#include <glib.h>
#include "mavconn.h"
#include "mavlinkros.h"

#include <sstream>

ros::Publisher mavlink_pub;

std::string lcmurl = "udpm://"; ///< host name for UDP server
bool verbose;

lcm_t *lcm;

static void
mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,
		const mavlink_message_t* msg, void * user)
{
	if (verbose)
		ROS_INFO("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n", msg->msgid, channel, msg->sysid, msg->compid);

	/**
	 * Serialize the Mavlink-ROS-message
	 */
	lcm_mavlink_ros::Mavlink rosmavlink_msg;
	createROSFromMavlink(msg,&rosmavlink_msg);

	/**
	 * Mark the ROS-Message as coming from LCM so that it will not be sent back to LCM
	 */
	rosmavlink_msg.fromlcm = true;

	/**
	 * Send the received MAVLink message to ROS (topic: mavlink, see main())
	 */
	mavlink_pub.publish(rosmavlink_msg);
}



void* lcm_wait(void* lcm_ptr) {
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1) {
		if (verbose) printf("LCM Handle...");
		lcm_handle(lcm);
	}
	return NULL;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lcmtoros");

	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "lcmurl", 'l', 0, G_OPTION_ARG_STRING, &lcmurl, "LCM Url to connect to", "udpm://" },
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- translate MAVLink messages from LCM to ROS (Topic: mavlink)");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}

	ros::NodeHandle n;

	mavlink_pub = n.advertise<lcm_mavlink_ros::Mavlink> ("mavlink", 1000);

	/**
	 * Connect to LCM Channel and register for MAVLink messages
	 */
	lcm_t* lcm = lcm_create(lcmurl.c_str());
	if (!lcm) {
		return 1;
	}

	mavlink_message_t_subscription_t * comm_sub = mavlink_message_t_subscribe(
			lcm, "MAVLINK", &mavlink_handler, NULL);

	while (ros::ok())
		lcm_handle(lcm);

	/*
	// Initialize LCM receiver thread
	GThread* lcm_thread;
	GError* err;

	if (!g_thread_supported()) {
		g_thread_init( NULL);
		// Only initialize g thread if not already done
	}

	if ((lcm_thread = g_thread_create((GThreadFunc) lcm_wait, (void *) lcm,
			TRUE, &err)) == NULL) {
		printf("Thread create failed: %s!!\n", err->message);
		g_error_free ( err);
	}*/

	//ros::Rate loop_rate(1);

	/**
	 * Now just wait until the process is terminated...
	 */
	//while (ros::ok()) {
	//	loop_rate.sleep();
	//}

	if (verbose) printf("Trying MAVLink unsubscribe");
	mavlink_message_t_unsubscribe (lcm, comm_sub);
	if (verbose) printf("Trying LCM destroy");
	lcm_destroy (lcm);
//	if (verbose) printf("Trying GThread Join");
//	g_thread_join(lcm_thread);

	return 0;
}

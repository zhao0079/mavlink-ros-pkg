/*=====================================================================
 
 MAVCONN Micro Air Vehicle Flying Robotics Toolkit
 
 (c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>
 
 This file is part of the MAVCONN project
 
 MAVCONN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 MAVCONN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.
 
 ======================================================================*/


/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device to the system's LCM bus.
 *
 *   @author Lorenz Meier, <mavteam@student.ethz.ch>
 *   @author Fabian Landau, <mavteam@student.ethz.ch>
 *   @author Benjamin Knecht (bknecht@student.ethz.ch)
 *
 */

#include "ros/ros.h"

#include "lcm_mavlink_ros/Mavlink.h"
#include "mavconn.h"
#include "mavlinkros.h"

#include <sstream>
#include <glib.h>

std::string lcmurl = "udpm://"; ///< host name for UDP server
bool verbose;

/**
 * Grabs all mavlink-messages from the ROS-Topic "mavlink" and publishes them on the LCM-Mavlink-Channel
 */

ros::Subscriber mavlink_sub;



// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// MAVLINK includes
#include <mavlink.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>
#include <glib.h>

namespace config = boost::program_options;
using std::string;
using namespace std;

struct timeval tv;		  ///< System time

int baud;                 ///< The serial baud rate

// Settings
int systemid = 42;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = PX_COMP_ID_MAVLINK_BRIDGE_SERIAL;
int serial_compid = 0;
std::string port = "/dev/ttyUSB0";              ///< The serial port name, e.g. /dev/ttyUSB0
bool silent = false;              ///< Wether console output should be enabled
bool verbose = true;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
bool test = false;                ///< Enable test mode
bool pc2serial = true;			  ///< Enable PC to serial push mode (send more stuff from pc over serial)

/**
 * @brief Handle a MAVLINK message received from LCM
 *
 * The message is forwarded to the serial port.
 *
 * @param rbuf LCM receive buffer
 * @param channel LCM channel
 * @param msg MAVLINK message
 * @param user LCM user
 */
static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,
							 const mavlink_message_t* msg, void * user)
{
	int fd = *(static_cast<int*>(user));
	if (fd == -1)
	{
		std::cerr << "Unable to send message over serial port. Port " << port << " not ready!" << std::endl;
	}
	else
	{
		//If msg not from system or not from IMU
		if (!pc2serial && (msg->sysid != systemid || msg->compid != serial_compid))
		{
				if (verbose || debug)
					std::cout << std::dec
					<< "Received and forwarded LCM message with id "
					<< static_cast<unsigned int> (msg->msgid)
					<< " from system " << static_cast<int> (msg->sysid)
					<< std::endl;
				
				// Send message over serial port
				uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
				int messageLength = mavlink_msg_to_send_buffer(buffer, msg);
				if (debug) printf("Writing %d bytes\n", messageLength);
				int written = write(fd, (char*)buffer, messageLength);
				//ioctl(fd, TIOCFLUSH, FWRITE);
				tcflush(fd, TCOFLUSH);
				if (messageLength != written) fprintf(stderr, "ERROR: Wrote %d bytes but should have written %d\n", written, messageLength);
		}
		
		if (pc2serial && msg->sysid == systemid)
		{
			if (verbose || debug)
				std::cout << std::dec
				<< "Received and forwarded LCM message with id "
				<< static_cast<unsigned int> (msg->msgid)
				<< " from system " << static_cast<int> (msg->sysid)
				<< std::endl;
			
			// Send message over serial port
			uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
			int messageLength = mavlink_msg_to_send_buffer(buffer, msg);
			if (debug) printf("Writing %d bytes\n", messageLength);
			int written = write(fd, (char*)buffer, messageLength);
			//ioctl(fd, TIOCFLUSH, FWRITE);
			tcflush(fd, TCOFLUSH);
			if (messageLength != written) fprintf(stderr, "ERROR: Wrote %d bytes but should have written %d\n", written, messageLength);
		}
		
		if (msg->msgid == MAVLINK_MSG_ID_PING)
		{
			mavlink_ping_t ping;
			mavlink_msg_ping_decode(msg, &ping);
			uint64_t r_timestamp = getSystemTimeUsecs();
			if (ping.target_system == 0 && ping.target_component == 0)
			{
				mavlink_message_t r_msg;
				mavlink_msg_ping_pack(systemid, compid, &r_msg, ping.seq, msg->sysid, msg->compid, r_timestamp);
				mavlink_message_t_publish(lcm, MAVLINK_MAIN, &r_msg);
			}
		}
	}
}

void* lcm_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		if (debug) printf("Waiting for LCM data\n");
		lcm_handle (lcm);
	}
	return NULL;
}


/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(std::string port)
{
	int fd; /* File descriptor for the port */
	
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
	
	return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;
	
	struct termios  config;
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %s is NOT a serial port\n", port.c_str());
		return false;
	}
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of port %s\n", port.c_str());
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;
	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0
	
	// Get the current options for the port
	//tcgetattr(fd, &options);
	
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, falling back to 115200 8N1 default rate.\n", baud);
			cfsetispeed(&config, B115200);
			cfsetospeed(&config, B115200);
			
			break;
	}
	
	/*
	 
	 //
	 // Enable the receiver and set local mode...
	 //
	 
	 options.c_cflag |= (CLOCAL | CREAD);
	 
	 // Setup 8N1
	 if (!parity)
	 {
	 options.c_cflag &= ~PARENB;
	 options.c_cflag &= ~CSTOPB;
	 }
	 
	 if (data_bits == 8)
	 {
	 options.c_cflag &= ~CSIZE;
	 options.c_cflag |= CS8;
	 }
	 
	 if (!hardware_control)
	 {
	 // Disable hardware flow control
	 //#ifdef _LINUX
	 options.c_cflag &= ~CRTSCTS;
	 //#endif
	 }
	 
	 // Choose raw input
	 options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	 
	 // Set one second timeout
	 options.c_cc[VMIN]  = 1;
	 options.c_cc[VTIME] = 10;
	 
	 // Set the new options for the port...
	 
	 tcsetattr(fd, TCSANOW, &options);
	 */
	
	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of port %s\n", port.c_str());
		return false;
	}
	return true;
}

void close_port(int fd)
{
	close(fd);
}

/**
 * @brief Serial function
 *
 * This function blocks waiting for serial data in it's own thread
 * and forwards the data once received.
 */
void* serial_wait(void* serial_ptr)
{
	int fd = *((int*) serial_ptr);
	
	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;
	
	// Blocking wait for new data
	while (1)
	{
		//if (debug) printf("Checking for new data on serial port\n");
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;
		//tcflush(fd, TCIFLUSH);
		if (read(fd, &cp, 1) > 0)
		{
			// Check if a message could be decoded, return the message in case yes
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
			if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
			{
				if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				if (debug)
				{
					unsigned char v=cp;
					fprintf(stderr,"%02x ", v);
				}
			}
			lastStatus = status;
		}
		else
		{
			if (!silent) fprintf(stderr, "ERROR: Could not read from port %s\n", port.c_str());
		}
		
		// If a message could be decoded, handle it
		if(msgReceived)
		{
			if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;
			
			// Do not send images over serial port
			
			// DEBUG output
			if (debug)
			{
				fprintf(stderr,"Forwarding SERIAL -> LCM: ");
				unsigned int i;
				uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
				unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
				if (messageLength > MAVLINK_MAX_PACKET_LEN)
				{
					fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
				}
				else
				{
					for (i=0; i<messageLength; i++)
					{
						unsigned char v=buffer[i];
						fprintf(stderr,"%02x ", v);
					}
					fprintf(stderr,"\n");
				}
			}
			
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
	}
	return NULL;
}


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

	if (verbose)
		ROS_INFO("Sent Mavlink from ROS to LCM, Message-ID: [%i]", mavlink_ros_msg->msgid);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rostolcm");

	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "lcmurl", 'l', 0, G_OPTION_ARG_STRING, &lcmurl, "LCM Url to connect to", "udpm://" },
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- translate MAVLink messages from ROS to LCM");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}
	
	
	// SETUP SERIAL PORT
	
	if (!silent) printf("SERIAL MAVLINK INTERFACE STARTED\n");
	
	// Exit if opening port failed
	// Open the serial port.
	if (!silent) printf("Trying to connect to %s.. ", port.c_str());
	int fd = open_port(port);
	if (fd == -1)
	{
		if (!silent) printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}
	if (!silent) printf("Trying to configure %s.. ", port.c_str());
	bool setup = setup_port(fd, baud, 8, 1, false, false);
	if (!setup)
	{
		if (!silent) printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}
	int* fd_ptr = &fd;
	
	
	GThread* serial_thread;
	GError* err;
	
	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}
	
	// Run indefinitely while the LCM and serial threads handle the data
	if (!silent) printf("\nREADY, waiting for serial/LCM data.\n");
	
	
	if( (serial_thread = g_thread_create((GThreadFunc)serial_wait, (void *)fd_ptr, TRUE, &err)) == NULL)
	{
		printf("Failed to create serial handling thread: %s!!\n", err->message );
		g_error_free ( err ) ;
	}
	
	int noErrors = 0;
	if (fd == -1 || fd == 0)
	{
		if (!silent) fprintf(stderr, "First attempt failed, waiting for port..\n");
	}
	else
	{
		if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", port.c_str(), baud);
	}
	
	// FIXME ADD MORE CONNECTION ATTEMPTS
	
	if(fd == -1 || fd == 0)
	{
		exit(noErrors);
	}
	
	// Ready to roll
	printf("\nMAVLINK TO ROS BRIDGE STARTED ON MAV %d (COMPONENT ID:%d) - RUNNING..\n\n", systemid, compid);
	
	

	ros::NodeHandle n;

	mavlink_sub = n.subscribe("mavlink", 1000, mavlinkCallback);

	

	/**
	 * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
	 */
	ros::spin();
	
	close_port(fd);
	
	g_thread_join(serial_thread);
	exit(0);

	return 0;
}

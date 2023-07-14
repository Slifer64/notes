#include <ros/ros.h>
#include <modbus/modbus.h>
#include <epickRTU.h>
#include "std_msgs/Int16.h"

#define cmd_speed_reg 0x2199
#define eta_speed_reg 0x219B
#define lfrd_reg 0x219A


int command;
RoboticEpic40 *gripper;


void commandCallback(const std_msgs::Int16::ConstPtr& msg)
{
	command = msg->data;
	// std::cout << "got command : " << command << std::endl; 

	if(command == 1) gripper->grip();
	else if(command == 0) gripper->release();
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "epick_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/epick", 1, commandCallback, ros::TransportHints().tcpNoDelay());

	std::string serial_filepath = "/tmp/ttyUR";

	gripper = new RoboticEpic40(serial_filepath);
	gripper->connect();

	// void RoboticEpic40::configure(int MOD,int PR,int SR,int FR)

	gripper->configure(1, 55, 20, 80);

	// ros::Rate loop_rate(10);

	ros::spin();
	gripper->exit();

	return 0;
}
#include <memory>
#include <ros/ros.h>
#include <modbus/modbus.h>
#include <atv12.h>
#include <std_msgs/Float32.h>


#define cmd_speed_reg 0x2199
#define eta_speed_reg 0x219B
#define lfrd_reg 0x219A


std::string receive_cmds_topic;
ros::Subscriber sub;
double conv_speed;


void readCommandCallback(const std_msgs::Float32::ConstPtr& msg)
{
	conv_speed = msg->data;
	std::cout << "conv_speed changed to: " << conv_speed << "\n";
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "run_conveyor_node");

	ros::NodeHandle nh("~");

	if (!nh.getParam("receive_cmds_topic", receive_cmds_topic)) throw std::runtime_error("Failed to read param \"receive_cmds_topic\"...");
	if (!nh.getParam("start_speed", conv_speed)) throw std::runtime_error("Failed to read param \"start_speed\"...");

	std::unique_ptr<atv12> drive;

	std::string serial_filepath = "/dev/ttyUSB0";
	try
	{
		drive.reset(new atv12(serial_filepath, 111, 19200, 'E', 8, 1));
		drive->connect();
		drive->init();
	}
	catch (const std::exception &e)
	{
		std::cerr << "\33[1m\33[31m" << e.what() << "\n" << "\33[0m" << std::flush;
		ros::shutdown();
		return 0;
	}

	sub = nh.subscribe(receive_cmds_topic, 1, readCommandCallback);
	
	// double conv_speed = argc < 2? 0.04: atof(argv[1]);

	while (ros::ok())
	{
		drive->setLinear(conv_speed);
		ros::spinOnce();
	}

	drive->setLinear(0);
	drive->exit();
	ros::shutdown();
	return 0;
}
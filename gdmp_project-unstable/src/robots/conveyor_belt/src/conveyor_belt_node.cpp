#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <atv12.h>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"

#define cmd_speed_reg 0x2199
#define eta_speed_reg 0x219B
#define lfrd_reg 0x219A

class ConveyorBeltControllerNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proximity_sub;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr robot_mode_sub;

  atv12 *drive;
  bool stop_conv;
  int robot_mode;

public:
  ConveyorBeltControllerNode() : Node("conveyor_belt_node")
  {
    /* Create Subscribers */
    this->proximity_sub  = this->create_subscription<std_msgs::msg::Bool>  ("/conveyor/proximity", 1, std::bind(&ConveyorBeltControllerNode::arduinoProximityCallback, this, std::placeholders::_1));
    this->robot_mode_sub = this->create_subscription<std_msgs::msg::Int16> ( "/robot_mode",        1, std::bind(&ConveyorBeltControllerNode::robotModeCallback,        this, std::placeholders::_1)); 
    //TODO:  ros::TransportHints().tcpNoDelay());

    /* Initialize Conveyor State */
    this->stop_conv = false;
    this->robot_mode = 0;

    /* Initialize Conveyor Obj */
    std::string serial_filepath_ = "/dev/ttyUSB0";
    this->drive = new atv12(serial_filepath_);
    this->drive->connect();
    this->drive->init();
    
    /* Set Initial velocity to ZERO */
    this->drive->setLinear(0);
  }

  ~ConveyorBeltControllerNode()
  {
    std::cout << "CONVEYOR BELT NODE SHUTTING DOWN" << std::endl;    
    this->drive->exit();
  }


  void robotModeCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void arduinoProximityCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

void ConveyorBeltControllerNode::arduinoProximityCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  /* Subscriber Callback of proximity_sub */
  this->stop_conv = msg->data;

  if (this->stop_conv || this->robot_mode != 4)
    this->drive->setRef(0);
  else
    this->drive->setLinear(-0.03);
}

void ConveyorBeltControllerNode::robotModeCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  /* Subscriber Callback of robot_mode_sub*/
  this->robot_mode = msg->data;
}

/*

  MAIN

*/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ConveyorBeltControllerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
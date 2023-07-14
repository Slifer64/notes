#include <online_adapt_controller/utils/gripper.h>

#include <std_msgs/Int16.h>

namespace online_adapt_
{

Gripper::Gripper(const std::string &gripper_topic)
{
  ctrl_pub = node.advertise<std_msgs::Int16>(gripper_topic, 1);
  is_open = false;
}

Gripper::~Gripper()
{

}

void Gripper::open()
{
  std_msgs::Int16 msg;
  msg.data = 0;
  ctrl_pub.publish(msg);
  is_open = true;
}

void Gripper::close()
{
  std_msgs::Int16 msg;
  msg.data = 1;
  ctrl_pub.publish(msg);
  is_open = false;
}

bool Gripper::isOpen() const
{
  return is_open;
}

} // namespace online_adapt_

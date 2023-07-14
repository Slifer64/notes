#ifndef $_PROJECT_384$_ONLINE_ADAPT_GRIPPER_H
#define $_PROJECT_384$_ONLINE_ADAPT_GRIPPER_H


#include <string>
#include <ros/ros.h>

namespace online_adapt_
{

class Gripper
{
public:

  Gripper(const std::string &gripper_topic);
  ~Gripper();

  void open();
  void close();

  bool isOpen() const;

private:

  ros::NodeHandle node;
  ros::Publisher ctrl_pub;
  bool is_open;
};


} // namespace online_adapt_

#endif // $_PROJECT_384$_ONLINE_ADAPT_GRIPPER_H

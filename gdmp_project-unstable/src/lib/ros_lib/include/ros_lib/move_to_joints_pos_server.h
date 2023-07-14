
#ifndef AS64_ROS_LIB_MOVE_TO_JOINTS_POS_SERVER_H
#define AS64_ROS_LIB_MOVE_TO_JOINTS_POS_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ros_lib/MoveToJointsPosAction.h>

#include <functional>
#include <vector>
#include <thread>

namespace as64_
{

class MoveToJointsPosActionServer
{
public:

  MoveToJointsPosActionServer(std::string name,
              std::function<void(const std::vector<double> &, double)> run_exec, 
              std::function<void()> stop_exec, 
              std::function<ros_lib::MoveToJointsPosFeedback()> get_feedback,
              std::function<ros_lib::MoveToJointsPosResult()> get_result);

  ~MoveToJointsPosActionServer(void);

  void executeCB(const ros_lib::MoveToJointsPosGoalConstPtr &goal);

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ros_lib::MoveToJointsPosAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  ros_lib::MoveToJointsPosFeedback feedback_;
  ros_lib::MoveToJointsPosResult result_;

  std::function<void(const std::vector<double> &, double)> run_exec_fun;
  std::function<void()> stop_exec_fun;
  std::function<ros_lib::MoveToJointsPosFeedback()> get_feedback_fun;
  std::function<ros_lib::MoveToJointsPosResult()> get_result_fun;
};

} // namespace as64_

#endif // AS64_ROS_LIB_MOVE_TO_JOINTS_POS_SERVER_H

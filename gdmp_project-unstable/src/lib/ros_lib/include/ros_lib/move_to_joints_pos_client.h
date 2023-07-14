
#ifndef AS64_ROS_LIB_MOVE_TO_JOINTS_POS_CLIENT_H
#define AS64_ROS_LIB_MOVE_TO_JOINTS_POS_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ros_lib/MoveToJointsPosAction.h>
#include <actionlib/client/terminal_state.h>

#include <ros/ros.h>

// using namespace ros_lib;

namespace as64_
{
  
  
class MoveToJointsPosActionClient
{
public:

  MoveToJointsPosActionClient(std::string name, bool spin_thread=true, bool wait_for_server=false);

  ~MoveToJointsPosActionClient(void);

  void sendGoal(const std::vector<double> &joints_pos);

  void sendGoal(const std::vector<double> &joints_pos, double duration);

  void cancel();

  bool waitFor(double t_sec);

  ros_lib::MoveToJointsPosFeedback feedback() const { return feedback_; }
  ros_lib::MoveToJointsPosResult result() const { return result_; }

protected:

  actionlib::SimpleActionClient<ros_lib::MoveToJointsPosAction> ac;
  
  ros_lib::MoveToJointsPosFeedback feedback_;
  ros_lib::MoveToJointsPosResult result_;

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const ros_lib::MoveToJointsPosResultConstPtr& result)
  {
    result_ = *result;
  }

  void activeCb()
  {}

  void feedbackCb(const ros_lib::MoveToJointsPosFeedbackConstPtr& feedback)
  {
    feedback_ = *feedback;
    // ROS_INFO("[MoveToJointsPosActionClient::feedbackCb] Progress: %.2f", progress);
  }
};

} // namespace as64_

#endif // AS64_ROS_LIB_MOVE_TO_JOINTS_POS_CLIENT_H

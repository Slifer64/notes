#ifndef ROBOT_CAMERA_TF_PUBLISHER_CONTROLLER_H
#define ROBOT_CAMERA_TF_PUBLISHER_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <robot_camera_tf_publisher/gui/gui.h>

#include <robot_wrapper/robot.h>
#include <robo_lib/robot_state_publisher.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <ros_lib/move_to_joints_pos_server.h>
#include <ros_lib/MoveToJointsPosFeedback.h>

using namespace as64_;

class Controller
{

public:
  Controller();
  ~Controller();

  void exec();

  void setMode(rw_::Mode mode);
  rw_::Mode getRobotMode() const { return robot->getMode(); }
  std::string getRobotModeName() const { return robot->getModeName(); }
  arma::vec getRobotJointsPosition() const { return robot->getJointsPosition(); }
  int getNumOfRobotJoints() const { return robot->getNumOfJoints(); }

  void publishRobotCameraTf() const;

  void printJointsPos() const;

  void move2JointPos(const arma::vec &joint_pos, double duration=-1);
  void move2JointPosRTthread(const arma::vec &joint_pos, double duration=-1);

  void enableAutoPublish(bool set);
  void vizPublishedTf(bool set) { viz_published_tf_in_rviz = set; }

  static void makeThreadRT(std::thread &thr);

private:

  std::shared_ptr<MoveToJointsPosActionServer> move_to_jpos_server;

  mutable tf::TransformBroadcaster tf_br;
  bool viz_published_tf_in_rviz;

  std::string child_frame_id; // the name of the child link for the published robot-camera transform
  std::string parent_frame_id; // the name of the parant link for the published robot-camera transform

  ros::Publisher tf_pub;
  bool run_auto_pub;
  double move_thres;
  unsigned check_move_rate_ms;

  // ========  Robot  ===========
  std::string robot_type;
  std::shared_ptr<rw_::Robot> robot;

  // ========  GUI ===========
  MainWindow *gui; // main gui window

  std::shared_ptr<robo_::RobotStatePublisher> rState_pub;
  void publishRobotState();

  static MainWindow *gui_;
  static void closeGUI(int);

  static arma::mat _quat2rotm(const arma::vec &quat);

  bool run_ = false;
  double exec_progress;
  void stopExec();
  ros_lib::MoveToJointsPosFeedback getExecFeedback() const { return exec_feedback; }
  ros_lib::MoveToJointsPosResult getExecResult() const { return exec_result; }
  ros_lib::MoveToJointsPosFeedback exec_feedback;
  ros_lib::MoveToJointsPosResult exec_result;
  ros::AsyncSpinner async_spinner;
};

#endif // ROBOT_CAMERA_TF_PUBLISHER_CONTROLLER_H

#ifndef $_PROJECT_384$_MAIN_CONTROLLER_H
#define $_PROJECT_384$_MAIN_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <std_msgs/Int16.h>

#include <main_controller/main_gui.h>
#include <main_controller/controller.h>
#include <main_controller/robot_joints_recorder/robot_joints_recorder.h>

#include <robo_lib/tool_estimator.h>
#include <robo_lib/robot_state_publisher.h>
#include <robo_lib/kinematic_chain.h>

#include <robot_wrapper/robot.h>
#include <robotiq_85_gripper/robotiq_85_gripper.h>
#include <apriltag_ros/apriltag_listener.h>
#include <rviz_lib/rviz_marker_publisher.h>

#include <main_controller/utils/gripper_control.h>

using namespace as64_;

class MainController
{

public:
  MainController();
  ~MainController();

  void exec() { launch(); gui_thr.join(); }

  arma::vec getStartPose() const;
  void setStartPose(const arma::vec &q);
  void setStartPoseSignal(const arma::vec &q);

  void sendEmergencyStopSignal() { emit gui->emergencyStopSignal(); }

  ExecResultMsg biasFTsensor() { robot->biasFTsensor(); return ExecResultMsg(ExecResultMsg::INFO, "FT-sensor biased!"); }

  void addController(Controller *ctrl);

public:

  enum RosAction
  {
    SET_MODE_IDLE = 0,
    SET_MODE_FREEDRIVE = 1,
    TRIGGER_EMERGENCY_STOP = 2,
    GOTO_START_POSE = 3,
    SET_CURRENT_POSE_AS_START = 4,
  };

  friend MainWindow;

  void setMode(rw_::Mode mode);
  rw_::Mode getMode() const { return robot->getMode(); }

  void setAdmittanceEnabledDoFs(const std::vector<bool> &enabled_dofs) { robot->adm_ctrl->setEnabledDoFs(enabled_dofs); }

  // ========  Robot  ===========
  std::string robot_type;
  std::shared_ptr<rw_::Robot> robot;
  std::shared_ptr<robo_::KinematicChain> robot_urdf;

  std::shared_ptr<robo_::ToolEstimator> tool_estimator;

  std::shared_ptr<r85_::R85Gripper> gripper;
  std::shared_ptr<GripperControl> gripper_ctrl;
  
  bool getCustomDigitalIn() const;
  std::function<bool()> get_custom_dig_in;

  // ========  GUI ===========
  MainWindow *gui; // main gui window
  std::thread gui_thr; // gui thread
  bool gui_finished;
  QMainWindow *createMainWindow();
  void launch();

  std::shared_ptr<robo_::RobotStatePublisher> rState_pub;
  void publishRobotState();

  arma::vec q_start;
  arma::vec start_pos;
  arma::vec start_quat;
  std::mutex q_start_mtx;

  bool goto_to_start_pose;
  ExecResultMsg gotoStartPose(bool joints_pos=true);
  ExecResultMsg moveToJointsPosition(const arma::vec &qT);
  ExecResultMsg moveToCartPose(const arma::vec &p_target, const arma::vec Q_target);
  ExecResultMsg setCurrentPoseAsStart();

  void printToolPose() const;
  void printJointsPosition() const;

  void viewStartPose(bool view);
  void viewStartJointPos(bool view);

  void makeThreadRT(std::thread &thr);

  std::vector< std::shared_ptr<Controller> > controller;

  std::shared_ptr<apriltag_ros::AprilTagListener> tag_listener;

  arma::mat getRobotCamTf() const
  {
    if (!is_T_robot_cam_updated)
      throw std::runtime_error("The robot-camera tf has not been updated...");
    return T_robot_cam;
  }

private:

  std::shared_ptr<rviz_::RvizMarkerPublisher> rviz_pub;

  std::string robot_desc_param;
  std::string base_link;
  std::string tool_link;
  std::string ftsensor_link;

  ros::NodeHandle node;
  ros::Subscriber ros_sub;
  void rosSubCallback(const std_msgs::Int16::ConstPtr& msg);

  ros::AsyncSpinner ros_async_spinner;

  static void closeGUI(int);
  static MainWindow *gui_; // used to emit closeGUI signal

  ros::Subscriber target_sub;
  void launchTargetPoseCb();

  std::shared_ptr<RobotJointsRecorder> joints_rec_;
  ExecResultMsg replayRecordedMotion();

  void loadCamraParams(const std::string &params_file);

  struct
  {
    arma::mat T_base_camOpt;
    arma::mat T_camBase_camOpt;
    std::string base_link;
    std::string camera_base_link;
    std::string camera_opt_frame_link;
  } camera_tf;

  std::string publish_robot_cam_tf_topic;
  

  void onRobotCameraCallibration();
  void onRobotCameraCallibrationCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg);
  void updateCameraPoseInRviz(const arma::mat &Tf_base_camOpt, const std::string &base, const std::string &cam_base);
  arma::mat robot_cam_tf;
  std::mutex robot_cam_tf_mtx;
  thr_::Semaphore robot_cam_tf_sem;
  std::string tag_detections_topic = "tag_detections";
  int on_robot_tag_id = 11;
  std::string robot_tag_link;
  arma::mat T_robot_cam;
  bool is_T_robot_cam_updated = false;
  std::string camera_base_link = "rs2_bottom_screw_frame";
  int tag_n = 0;
  arma::vec tag_pos;
  arma::vec tag_quat;
  arma::vec prev_quat;
};

#endif // $_PROJECT_384$_MAIN_CONTROLLER_H

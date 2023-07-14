#include <iostream>
#include <csignal>
#include <iomanip>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


#include <QApplication>

#include <robot_wrapper/lwr4p_robot.h>
#include <robot_wrapper/ur_robot.h>

#include <robot_camera_tf_publisher/controller.h>
#include <robot_camera_tf_publisher/print_utils.h>

#include <Eigen/Dense>


using namespace as64_;

#define Controller_fun_ std::string("[Controller::") + __func__ + "]: "

MainWindow *Controller::gui_ = 0;

arma::mat Controller::_quat2rotm(const arma::vec &quat)
{
  double qw=quat(0), qx=quat(1), qy=quat(2), qz=quat(3);

  arma::mat rotm;
  rotm = {{1 - 2*qy*qy - 2*qz*qz,      2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw},
	        {    2*qx*qy + 2*qz*qw,  1 - 2*qx*qx - 2*qz*qz,      2*qy*qz - 2*qx*qw},
	        {    2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,  1 - 2*qx*qx - 2*qy*qy}};
  // rotm << 1 - 2*qy*qy - 2*qz*qz << 	2*qx*qy - 2*qz*qw     <<  	2*qx*qz + 2*qy*qw << arma::endr
	//      << 2*qx*qy + 2*qz*qw     <<  1 - 2*qx*qx - 2*qz*qz <<  	2*qy*qz - 2*qx*qw << arma::endr
	//      << 2*qx*qz - 2*qy*qw     <<    2*qy*qz + 2*qx*qw   << 	1 - 2*qx*qx - 2*qy*qy;

  return rotm;
}

Controller::Controller() : async_spinner(1)
{
  ros::NodeHandle nh("~");

  std::string tf_publish_topic;
  if (!nh.getParam("tf_publish_topic", tf_publish_topic)) throw std::ios_base::failure(Controller_fun_ + "Failed to read \"tf_publish_topic\" param.");

  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure(Controller_fun_ + "Failed to read \"robot_type\" param.");
  std::vector<double> q_start2;
  bool use_sim;
  if (!nh.getParam("use_sim", use_sim)) throw std::ios_base::failure(Controller_fun_ + "Failed to read \"use_sim\" param.");

  std::string robot_desc;
  std::string base_link;
  std::string tool_link;
  if (!nh.getParam("robot_description_name",robot_desc)) throw std::ios_base::failure(Controller_fun_ + "Failed to read parameter \"robot_description_name\".");
  if (!nh.getParam("base_link",base_link)) throw std::ios_base::failure(Controller_fun_ + "Failed to read parameter \"base_link\".");
  if (!nh.getParam("camera_color_optical_link",tool_link)) throw std::ios_base::failure(Controller_fun_ + "Failed to read parameter \"camera_color_optical_link\".");

  std::string move2jpos_topic;
  if (!nh.getParam("move2jpos_topic",move2jpos_topic)) move2jpos_topic = "";

  child_frame_id = tool_link;
  parent_frame_id = base_link;

  if (robot_type.compare("lwr4p")==0) 
  {
    double ctrl_cycle;
    if (!nh.getParam("ctrl_cycle",ctrl_cycle)) throw std::ios_base::failure(Controller_fun_ + "Failed to read parameter \"ctrl_cycle\".");

    robot.reset(new rw_::LWR4p_Robot(use_sim, robot_desc, base_link, tool_link, tool_link, ctrl_cycle));
  }
  else if (robot_type.compare("ur")==0)
  {
    std::string robot_ip;
    std::string host_ip;
    int reverse_port;

    if (use_sim) robot_ip = "127.0.0.1";
    else if (!nh.getParam("robot_ip",robot_ip)) throw std::ios_base::failure(Controller_fun_ + "Failed to read parameter \"robot_ip\".");

    if (use_sim) host_ip = "127.0.0.1";
    else if (!nh.getParam("host_ip",host_ip)) throw std::ios_base::failure(Controller_fun_ + "Failed to read parameter \"host_ip\".");

    if (!nh.getParam("reverse_port",reverse_port))
    {
      reverse_port = 8080;
      PRINT_WARNING_MSG(std::string(Controller_fun_ + "Failed to read parameter \"reverse_port\".\n") + "Setting default: reverse_port = 8080\n");
    }
    robot.reset(new rw_::Ur_Robot(robot_desc, base_link, tool_link, tool_link, robot_ip, host_ip, reverse_port));
    // robot.reset(new rw_::Ur_Robot(use_sim));
  }
  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");

  if (!move2jpos_topic.empty())
  {
    move_to_jpos_server.reset(new MoveToJointsPosActionServer(move2jpos_topic,
              std::bind(&Controller::move2JointPosRTthread, this, std::placeholders::_1, std::placeholders::_2), 
              std::bind(&Controller::stopExec, this), 
              std::bind(&Controller::getExecFeedback, this),
              std::bind(&Controller::getExecResult, this)));
  }
  
  // std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // why ??

  publishRobotState();

  // register signal SIGINT and signal handler
  signal(SIGINT, closeGUI);
  signal(SIGINT, Controller::closeGUI);

  tf_pub = nh.advertise<geometry_msgs::TransformStamped>(tf_publish_topic, 1);
  run_auto_pub = false;
  move_thres = 5e-4;
  check_move_rate_ms = 33;
  viz_published_tf_in_rviz = false;

  publishRobotCameraTf();

  async_spinner.start();
}

Controller::~Controller()
{
  run_auto_pub = false;
  if (robot->isOk()) robot->stop();
  if (rState_pub) rState_pub->stop();
}

void Controller::publishRobotState()
{
  std::string robot_desc;
  if (!ros::NodeHandle("~").getParam("robot_description_name", robot_desc)) throw std::ios_base::failure(Controller_fun_ + "Failed to read \"robot_description\" param.");

  rState_pub.reset(new robo_::RobotStatePublisher( robot_desc, robot->getJointNames(), [this](){ return robot->getJointsPosition(); } ) );
  rState_pub->setPublishCycle(33);
  rState_pub->start();
}

void Controller::publishRobotCameraTf() const
{
  arma::vec pos = robot->getTaskPosition();
  arma::vec quat = robot->getTaskOrientation();

  arma::mat Tf = arma::join_horiz(this->_quat2rotm(quat), pos);
  for (int i=0; i<Tf.n_rows; i++)
  {
    std::cout << "[ ";
    for (int j=0; j<Tf.n_cols; j++) std::cout << std::setprecision(6) << Tf(i,j) << " ,  ";
    std::cout << "]\n";
  }
  // std::cout << "Tf = \n" << std::setprecision(6) << Tf << "\n";

  // -------------------------------

  geometry_msgs::TransformStamped Tf_stamp;

  Tf_stamp.header.frame_id = parent_frame_id;
  Tf_stamp.header.stamp = ros::Time::now();

  Tf_stamp.child_frame_id = child_frame_id;

  Tf_stamp.transform.translation.x = pos(0);
  Tf_stamp.transform.translation.y = pos(1);
  Tf_stamp.transform.translation.z = pos(2);

  Tf_stamp.transform.rotation.w = quat(0);
  Tf_stamp.transform.rotation.x = quat(1);
  Tf_stamp.transform.rotation.y = quat(2);
  Tf_stamp.transform.rotation.z = quat(3);

  tf_pub.publish(Tf_stamp);

  if (viz_published_tf_in_rviz)
  {
    Tf_stamp.child_frame_id = "camera_frame"; // change the name, because the tf of the tool_link is already published
    tf_br.sendTransform(Tf_stamp);
  }

  // -------------------------------
  // geometry_msgs::Pose pose;

  // pose.position.x = pos(0);
  // pose.position.y = pos(1);
  // pose.position.z = pos(2);

  // pose.orientation.w = quat(0);
  // pose.orientation.x = quat(1);
  // pose.orientation.y = quat(2);
  // pose.orientation.z = quat(3);

  // tf_pub.publish(pose);
}

void Controller::printJointsPos() const
{
  arma::vec joint_pos = robot->getJointsPosition();
  std::ostringstream oss;
  oss.precision(3);
  oss.width(4);
  oss << "robot joints position: [";
  for (int i=0; i<joint_pos.size()-1; i++) oss << joint_pos[i] << ", ";
  oss << joint_pos.back() << "]\n";
  PRINT_INFO_MSG(oss.str());
  std::cerr << "\33[1m\33[34m" << oss.str() << "\33[0m" << std::flush;
}

void Controller::closeGUI(int)
{
  emit Controller::gui_->closeSignal();
}

void Controller::exec()
{
  int argc = 0;
  char **argv = 0;
  QApplication app(argc, argv);
  // QThread::currentThread()->setPriority(QThread::LowestPriority);

  gui = new MainWindow(this);
  Controller::gui_ = gui;

  gui->show();
  app.exec();

  delete gui; // must be destructed in this thread!
}

void Controller::setMode(rw_::Mode mode)
{
  if (robot->isOk())
  {
    this->robot->setMode(mode);
    if (robot->getMode() == mode)   emit gui->modeChangedSignal();
  }
}

void Controller::stopExec()
{
  run_ = false;
}

void Controller::move2JointPosRTthread(const arma::vec &joint_pos, double duration)
{
  std::thread thr = std::thread([this, joint_pos, duration](){ this->move2JointPos(joint_pos, duration); });
  makeThreadRT(thr);
  thr.detach();
}


void Controller::move2JointPos(const arma::vec &joint_pos, double duration)
{
  run_ = true;

  rw_::Mode prev_mode = robot->getMode(); // store current robot mode
  this->setMode(rw_::JOINT_POS_CONTROL);

  robot->update(); // waits for the next tick

  arma::vec q0 = robot->getJointsPosition();
  arma::vec qref = q0;
  arma::vec qT = joint_pos;
  arma::vec q_diff = (qT - q0);
  q_diff.back() *= 0.5; // give less weight to last joint
  if (duration < 0) duration = std::max(arma::max(arma::abs(q_diff)) * 10.0 / 3.14159, 2.5);
  double t = 0.0;

  exec_feedback.finished = false;
  exec_feedback.current_joint_pos = std::vector<double>(q0.memptr(), q0.memptr() + q0.size());
  exec_feedback.progress = t/duration;

  exec_result.success = true;

  while ( t < duration && run_)
  {
    if ( robot->getMode() != rw_::JOINT_POS_CONTROL )
    {
      exec_feedback.finished = true;
      exec_result.success = false;
      exec_result.msg = "The robot changed mode...\nAborting...";
      emit gui->showMsgSignal(exec_result.msg.c_str(), WARN_MSG);
      return;
    }
    
    if (!robot->isOk())
    {
      exec_feedback.finished = true;
      exec_result.success = false;
      exec_result.msg = "The robot is not ok...\nAborting...";
      emit gui->showMsgSignal(exec_result.msg.c_str(), ERR_MSG);
      return;
    }

    t += robot->getCtrlCycle();
    qref = rw_::get5thOrder(t, q0, qT, duration).col(0);
    robot->setJointsPosition(qref);

    robot->update(); // waits for the next tick

    arma::vec q = robot->getJointsPosition();
    exec_feedback.current_joint_pos = std::vector<double>(q.memptr(), q.memptr() + q.size());
    exec_feedback.progress = std::max(t/duration, 1.0);
  }

  this->setMode(prev_mode);

  exec_feedback.finished = true;

  exec_result.msg = "Reached joints position!";
  PRINT_INFO_MSG(exec_result.msg);
  // emit gui->showMsgSignal(exec_result.msg.c_str(), INFO_MSG);
}

void Controller::makeThreadRT(std::thread &thr)
{
  struct sched_param sch_param;
  int prev_policy;
  pthread_getschedparam(thr.native_handle(), &prev_policy, &sch_param);
  sch_param.sched_priority = 99;
  int ret_code = pthread_setschedparam(thr.native_handle(), SCHED_FIFO, &sch_param);

  std::function<std::string(int)> setThreadPriorErrMsg = [](int err_code)
  {
    switch (err_code)
    {
      case ESRCH:
        return "No thread with the ID thread could be found.";
      case EINVAL:
        return "Policy is not a recognized policy, or param does not make sense for the policy.";
      case EPERM:
        return "The caller does not have appropriate privileges to set the specified scheduling policy and parameters.";
      case ENOTSUP:
        return "Attempt was made to set the policy or scheduling parameters to an unsupported value.";
      default:
        return "Unknown error code.";
    }
  };

  if (ret_code) PRINT_WARNING_MSG(Controller_fun_ + "Failed to set thread priority! Reason:\n" + setThreadPriorErrMsg(ret_code) + "\n", std::cerr);
  else PRINT_INFO_MSG(Controller_fun_ + "Set thread priority successfully!\n", std::cerr);
}

void Controller::enableAutoPublish(bool set)
{
  // Disable auto-publish
  if (!set)
  {
    run_auto_pub = false;
    return;
  }
  
  // If already running, ignore the signal...
  if (run_auto_pub) return;

  // Make the flag true and launch the thread
  run_auto_pub = true;
  std::thread([this]()
  {
    arma::vec j0 = robot->getJointsPosition();

    while (run_auto_pub)
    {
      arma::vec j1 = robot->getJointsPosition();

      if ( arma::norm(j1 - j0) > move_thres )
      {
        j0 = j1;
        publishRobotCameraTf();
      }
      std::this_thread::sleep_for( std::chrono::milliseconds(check_move_rate_ms) );
    }
  }).detach();

}
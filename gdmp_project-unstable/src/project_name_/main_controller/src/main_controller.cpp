#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

#include <robot_wrapper/lwr4p_robot.h>
#include <robot_wrapper/ur_robot.h>

#include <plot_lib/qt_plot.h>
#include <gui_lib/utils.h>
#include <gui_lib/gui_application.h>

#include <robo_lib/trajectory.h>
#include <robo_lib/singular_value_filter.h>

#include <math_lib/quaternions.h>

#include <main_controller/main_controller.h>

#include <yaml-cpp/yaml.h>

using namespace as64_;

#define MainController_fun_ std::string("[MainController::") + __func__ + "]: "

MainWindow *MainController::gui_ = 0;


MainController::MainController() : ros_async_spinner(2)
{
  this->gui = NULL;

  // Gb_::main_ctrl = this;

  ros::NodeHandle nh("~");

  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure(MainController_fun_ + "Failed to read \"robot_type\" param.");

  if (!nh.getParam("robot_description_name", this->robot_desc_param)) throw std::runtime_error("Failed to load param \"robot_description_name\"...");
  if (!nh.getParam("base_link", this->base_link)) throw std::runtime_error("Failed to load param \"base_link\"...");
  if (!nh.getParam("tool_link", this->tool_link)) throw std::runtime_error("Failed to load param \"tool_link\"...");
  if (!nh.getParam("ftsensor_link", this->ftsensor_link)) ftsensor_link = tool_link;
  robot_urdf.reset(new robo_::KinematicChain);
  robot_urdf->initFromParam(this->robot_desc_param, this->base_link, this->tool_link);


  if (!nh.getParam("robot_tag_link", this->robot_tag_link)) this->robot_tag_link = "ur5_apriltag_link";
  nh.getParam("robot_cam_tf_topic", this->publish_robot_cam_tf_topic);
  std::string robot_cam_tf_params;
  if (nh.getParam("robot_cam_tf_params", robot_cam_tf_params)) loadCamraParams(robot_cam_tf_params);
  
  
  std::vector<double> q_start2;
  if (!nh.getParam("q_start", q_start2)) throw std::ios_base::failure(MainController_fun_ + "Failed to read \"q_start\" param.");
  this->setStartPose(q_start2);

  std::vector<double> start_pose;
  if (nh.getParam("start_pose", start_pose))
  {
    this->start_pos = {start_pose[0], start_pose[1], start_pose[2]};
    this->start_quat = {start_pose[3], start_pose[4], start_pose[5], start_pose[6]};
  }
  else
  {
    this->start_pos = robot_urdf->getPosition(this->q_start);
    this->start_quat = robot_urdf->getQuat(this->q_start);
  }

  bool use_sim;
  if (!nh.getParam("use_sim", use_sim)) throw std::ios_base::failure(MainController_fun_ + "Failed to read \"use_sim\" param.");

  if (nh.hasParam("r85_gripper_options"))
  {
    gripper.reset(new r85_::R85Gripper("r85_gripper_options"));
    gripper_ctrl.reset(new GripperControl(gripper.get(), std::bind(&MainController::getCustomDigitalIn, this)));
    gripper_ctrl->launchControlThread();
  }

  if (robot_type.compare("lwr4p")==0) robot.reset(new rw_::LWR4p_Robot(use_sim));
  else if (robot_type.compare("ur")==0)
  {
    robot.reset(new rw_::Ur_Robot(use_sim));
    get_custom_dig_in = [this](){ return (dynamic_cast<rw_::Ur_Robot*>(robot.get())->getDigitalIn()[0] > 0); };
  }

  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");

  std::vector<double> Fext_dead_zone;
  if (nh.getParam("Fext_dead_zone", Fext_dead_zone)) robot->setWrenchDeadZone(arma::vec(Fext_dead_zone));
  else robot->setWrenchDeadZone({0.1,0.1,0.1, 0.01,0.01,0.01});

  publishRobotState();

  // =======  load default paths for data IO  =======

  // std::string main_path = ros::package::getPath(PROJECT_NAME) + "/";

  // register signal SIGINT and signal handler
  signal(SIGINT, MainController::closeGUI);

//  // model.reset(new Model());
//  std::string model_filename;
//  if (!nh.getParam("model_filename", model_filename)) PRINT_WARNING_MSG("Failed to load model...\n");
//  else model.loadModel(model->getDefaultPath() + model_filename);
//  std::cerr << "============> Model loaded successfully!\n";
//
//  train_ctrl.reset(new TrainController(this, robot.get(), model));
//  exec_ctrl.reset(new ExecController(this, robot.get(), model));
//
//  gmp_mpc_ctrl.reset(new GmpMpcController(this, robot.get()));

  rviz_pub.reset( new rviz_::RvizMarkerPublisher("/" + getNodeNameID() + "_main_ctrl_marker_topic", this->base_link) );

  joints_rec_.reset(new RobotJointsRecorder(robot.get()));
  //joints_rec_->setDefaultPath(exec_ctrl->getDefaultPath());
  try { joints_rec_->loadData(joints_rec_->getDefaultPath() + "joints_rec_data.bin"); }
  catch (std::exception &e) { PRINT_WARNING_MSG(std::string("[Load recorded joints]:") + e.what() + "\n"); }

  ros_sub = node.subscribe("main_controller_ros_actions", 1, &MainController::rosSubCallback, this);
  ros_async_spinner.start();

  if (nh.hasParam("apriltag_listener_options")) tag_listener.reset(new apriltag_ros::AprilTagListener("apriltag_listener_options"));
}

MainController::~MainController()
{
  if (robot->isOk()) robot->stop();

  if (rState_pub) rState_pub->stop();

  std::cerr << "[MainController::~MainController]: Waiting to be notified...\n";
  while (!gui_finished); // wait for gui to finish
  std::cerr << "[MainController::~MainController]: Got notification!\n";
}

void MainController::rosSubCallback(const std_msgs::Int16::ConstPtr& msg)
{
  switch (static_cast<MainController::RosAction>(msg->data))
  {
    case MainController::SET_MODE_IDLE:
      gui->trigger_idleBtnPressed();
      break;
    case MainController::SET_MODE_FREEDRIVE:
      gui->trigger_freedriveBtnPressed();
      break;
    case MainController::TRIGGER_EMERGENCY_STOP:
      gui->trigger_emergencyStopBtnPressed();
      break;
    case MainController::GOTO_START_POSE:
      gui->trigger_gotoStartPoseBtnPressed();
      break;
    case MainController::SET_CURRENT_POSE_AS_START:
      gui->trigger_setCurrentPoseAsStartBtnPressed();
      break;
    default:
      PRINT_WARNING_MSG(MainController_fun_ + "Unrecognized input action...\n");
  }
}

bool MainController::getCustomDigitalIn() const
{
  if (!get_custom_dig_in)
  {
    PRINT_WARNING_MSG(MainController_fun_ + "Not implemented...\n");
    return false;
  }
  return get_custom_dig_in();
}

void MainController::loadCamraParams(const std::string &path)
{
  std::size_t found = path.find_first_of("/");
  if (found == std::string::npos) throw std::runtime_error(MainController_fun_ + "camera params path must be <package>/<params_file_path_in_package>");

  std::string package_name = path.substr(0, found);
  std::string params_file = path.substr(found+1);

  params_file = ros::package::getPath(package_name) + params_file;

  YAML::Node nh;
  
  try
  {
    nh = YAML::LoadFile(params_file);
  }
  catch(const std::exception& e)
  {
    PRINT_WARNING_MSG("Camera params file '" + params_file + "' read error:\n" + e.what());
    return;
  }

  arma::vec pos;
  arma::vec quat;

  YAML::Node n_T_base_camOpt;
  if (!YAML::getParam(nh, "T_base_camOpt", n_T_base_camOpt)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'T_base_camOpt'...\n");
  if (!YAML::getParam(n_T_base_camOpt, "pos", pos)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'T_base_camOpt.pos'...\n");
  if (!YAML::getParam(n_T_base_camOpt, "orient", quat)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'T_base_camOpt.orient'...\n");
  camera_tf.T_base_camOpt = arma::mat().eye(4,4);
  camera_tf.T_base_camOpt.submat(0, 3, 2, 3) = pos;
  camera_tf.T_base_camOpt.submat(0, 0, 2, 2) = math_::quat2rotm(quat);

  YAML::Node n_T_camBase_camOpt;
  if (!YAML::getParam(nh, "T_camBase_camOpt", n_T_camBase_camOpt)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'T_camBase_camOpt'...\n");
  if (!YAML::getParam(n_T_camBase_camOpt, "pos", pos)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'T_camBase_camOpt.pos'...\n");
  if (!YAML::getParam(n_T_camBase_camOpt, "orient", quat)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'T_camBase_camOpt.orient'...\n");
  camera_tf.T_camBase_camOpt = arma::mat().eye(4,4);
  camera_tf.T_camBase_camOpt.submat(0, 3, 2, 3) = pos;
  camera_tf.T_camBase_camOpt.submat(0, 0, 2, 2) = math_::quat2rotm(quat);

  if (!YAML::getParam(nh, "base_link", camera_tf.base_link)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'base_link'...\n");
  if (!YAML::getParam(nh, "camera_base_link", camera_tf.camera_base_link)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'camera_base_link'...\n");
  if (!YAML::getParam(nh, "camera_opt_frame_link", camera_tf.camera_opt_frame_link)) throw std::runtime_error(MainController_fun_ + "Failed to read param 'camera_opt_frame_link'...\n");

  updateCameraPoseInRviz(camera_tf.T_base_camOpt, camera_tf.base_link, camera_tf.camera_base_link);
}

void MainController::addController(Controller *ctrl)
{
  controller.push_back( std::shared_ptr<Controller>(ctrl) );
}

void MainController::publishRobotState()
{
  std::string robot_desc;
  if (!ros::NodeHandle("~").getParam("robot_description_name", robot_desc)) throw std::ios_base::failure(MainController_fun_ + "Failed to read \"robot_description\" param.");

  std::vector<std::string> joint_names = robot->getJointNames();
  std::function<arma::vec()> getJointsPosFun;
  if (gripper)
  {
    std::vector<std::string> joint_names2 = gripper->getJointNames();
    for (int i=0;i<joint_names2.size(); i++) joint_names.push_back(joint_names2[i]);
    getJointsPosFun = [this]()
      {
        arma::vec j = arma::join_vert(robot->getJointsPosition(), gripper->getJointsPosition());
        return j;
      };
  }
  else getJointsPosFun = [this]() { return robot->getJointsPosition(); };

  rState_pub.reset(new robo_::RobotStatePublisher(robot_desc, joint_names, getJointsPosFun));
  rState_pub->setPublishCycle(33);
  rState_pub->start();
}

void MainController::closeGUI(int)
{
  emit MainController::gui_->closeSignal();
}

QMainWindow *MainController::createMainWindow()
{
  gui = new MainWindow(this);
  MainController::gui_ = gui;
  return gui;
}

void MainController::launch()
{
  gui_finished = false;
  gui_thr = std::thread([this]()
  {
    // gui_::launchGui(std::bind(&MainController::createMainWindow, this), &gui_finished, QThread::LowestPriority, [](){ pl_::QtPlot::init(); });
    gui_finished = false;

    int argc = 0;
    char **argv = 0;
    gui_::GuiApplication app(argc, argv);
    QThread::currentThread()->setPriority(QThread::LowestPriority);

    gui = new MainWindow(this);
    MainController::gui_ = gui;
    pl_::QtPlot::init();
    gui->show();
    app.exec();

    delete gui; // must be destructed in this thread!
    gui_finished = true;
  });

  int err_code = thr_::setThreadPriority(gui_thr, SCHED_OTHER, 0);
  if (err_code) PRINT_WARNING_MSG("[MainController::launch]: Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG("[MainController::launch]: Set thread priority successfully!\n", std::cerr);
}

void MainController::setMode(rw_::Mode mode)
{
  if (robot->isOk())
  {
    this->robot->setMode(mode);
    if (robot->getMode() == mode) emit gui->modeChangedSignal();
  }
}

ExecResultMsg MainController::gotoStartPose(bool joints_pos)
{
  if (joints_pos) return moveToJointsPosition(this->getStartPose());
  else return moveToCartPose(start_pos, start_quat);
}

ExecResultMsg MainController::moveToJointsPosition(const arma::vec &qT)
{
  rw_::Mode prev_mode = robot->getMode(); // store current robot mode
  this->setMode(rw_::JOINT_POS_CONTROL);

  robot->update(); // waits for the next tick

  arma::vec q0 = robot->getJointsPosition();

  if (arma::max(arma::abs(q0 - qT)) < 1e-3)
    return ExecResultMsg(ExecResultMsg::INFO, "Already at target!");

  arma::vec qref = q0;
  arma::vec q_diff = (qT - q0);
  q_diff.back() *= 0.5; // give less weight to last joint
  double duration = std::max(arma::max(arma::abs(q_diff)) * 10.0 / 3.14159, 2.5);
  double t = 0.0;
  while (t < duration)
  {
    if (!robot->isOk()) return ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\nOperation aborted...");

    if (robot->getMode() != rw_::JOINT_POS_CONTROL) return ExecResultMsg(ExecResultMsg::WARNING, "The robot mode changed...\nOperation stopped...");

    t += robot->getCtrlCycle();
    qref = rw_::get5thOrder(t, q0, qT, duration).col(0);
    robot->setJointsPosition(qref);

    robot->update(); // waits for the next tick
  }
  this->setMode(prev_mode);

  double err = arma::max(arma::abs(robot->getJointsPosition() - qT));
  if (err > 1e-3)
    return ExecResultMsg(ExecResultMsg::WARNING, "Failed to reach start pose: max_joint_error=" + std::to_string(err));
  else
    return ExecResultMsg(ExecResultMsg::INFO, "Reached target!");

}

ExecResultMsg MainController::moveToCartPose(const arma::vec &p_target, const arma::vec Q_target)
{
  rw_::Mode prev_mode = robot->getMode(); // store current robot mode

  auto ctrl_mode = rw_::JOINT_POS_CONTROL;
  // auto ctrl_mode = rw_::CART_VEL_CTRL;
  this->setMode(ctrl_mode);

  robot->update(); // waits for the next tick

  arma::vec p0 = robot->getTaskPosition();
  arma::vec Q0 = robot->getTaskOrientation();
  double duration = std::max(arma::norm(p0 - p_target)*7.5, 2.5);
  double t = 0.0;

  if (arma::norm(p0 - p_target) < 1e-3 && arma::norm(math_::quatLog(math_::quatDiff(Q0, Q_target))) < 1e-2)
    return ExecResultMsg(ExecResultMsg::INFO, "Already at target!");

  arma::vec j0 = robot->getJointsPosition();

  while (t < duration)
  {
    if (!robot->isOk()) return ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\nOperation aborted...");

    if (robot->getMode() != ctrl_mode) return ExecResultMsg(ExecResultMsg::WARNING, "The robot mode changed...\nOperation stopped...");

    t += robot->getCtrlCycle();
    robo_::TrajPoint p = robo_::get5thOrderTraj(t, p0, p_target, duration);
    robo_::QuatTrajPoint q = robo_::get5thOrderQuatTraj(t, Q0, Q_target, duration);

    arma::vec V_ref = arma::join_vert(p.vel, q.rot_vel);

    if (ctrl_mode == rw_::CART_VEL_CTRL) robot->setTaskVelocity(V_ref, p.pos, q.quat);
    else if (ctrl_mode == rw_::JOINT_POS_CONTROL)
    {
      arma::mat pose = arma::mat().eye(4, 4);
      pose.submat(0, 3, 2, 3) = p.pos;
      pose.submat(0, 0, 2, 2) = math_::quat2rotm(q.quat); 
      bool found_solution = false;
      arma::vec j_pos_cmd = robot_urdf->getJointPositions(pose, j0, &found_solution);

      if (!found_solution)
      {
        this->setMode(prev_mode);
        return ExecResultMsg(ExecResultMsg::WARNING, "Failed to find inverse kinematic solution...");
      }

      j0 = j_pos_cmd;
      robot->setJointsPosition(j_pos_cmd);
    }

    robot->update(); // waits for the next tick
  }
  this->setMode(prev_mode);

  double pos_err = arma::norm(robot->getTaskPosition() - p_target);
  double orient_err = arma::norm(math_::quatLog(math_::quatDiff(robot->getTaskOrientation(), Q_target)));
  if (pos_err < 1e-3 && orient_err<1e-2)
    return ExecResultMsg(ExecResultMsg::INFO, "Reached target!");
  else
    return ExecResultMsg(ExecResultMsg::WARNING, 
        "Failed to reach start pose: pos_err=" + std::to_string(pos_err) + ", orient_err=" + std::to_string(orient_err));
}

ExecResultMsg MainController::setCurrentPoseAsStart()
{
  this->setStartPose(robot->getJointsPosition());
  return ExecResultMsg(ExecResultMsg::INFO, "Registered current pose as start!");
}

void MainController::setStartPose(const arma::vec &q)
{
  std::unique_lock<std::mutex> lck(this->q_start_mtx);
  this->q_start=q;
  if (robot_urdf)
  {
    this->start_pos = robot_urdf->getPosition(q);
    this->start_quat = robot_urdf->getQuat(q);
  }
  else PRINT_WARNING_MSG(MainController_fun_ + "robot_urdf is not initialized. Skipping initialization of start_pos and start_quat...\n");
}

void MainController::setStartPoseSignal(const arma::vec &q)
{
  setStartPose(q);
  if (this->gui) emit gui->startPoseChangedSignal(q, ExecResultMsg(ExecResultMsg::INFO, "Start pose changed!"));
  else PRINT_WARNING_MSG(MainController_fun_ + "Failed to sent \"startPoseChangedSignal\": the gui is not initialized...\n");
}

arma::vec MainController::getStartPose() const
{
  std::unique_lock<std::mutex> lck(const_cast<MainController*>(this)->q_start_mtx);
  return this->q_start;
}

void MainController::makeThreadRT(std::thread &thr)
{
  int err_code = thr_::setThreadPriority(thr, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG("[MainController::makeThreadRT]: Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG("[MainController::makeThreadRT]: Set thread priority successfully!\n", std::cerr);
}

void MainController::printToolPose() const
{
  arma::vec P = this->robot->getTaskPosition();
  arma::vec Q = this->robot->getTaskOrientation();
  arma::vec pose = arma::join_vert(P, Q);
  std::ostringstream oss;
  oss << "robot tool pose: [";
  oss.precision(3);
  oss.width(4);
  for (int i=0; i<pose.size()-1; i++) oss << pose[i] << "; ";
  oss << pose.back() << "]\n";
  PRINT_INFO_MSG(oss.str());
}

void MainController::printJointsPosition() const
{
  arma::vec joint_pos = this->robot->getJointsPosition();
  std::ostringstream oss;
  oss.precision(3);
  oss.width(4);
  oss << "robot joints position: [";
  for (int i=0; i<joint_pos.size()-1; i++) oss << joint_pos[i] << ", ";
  oss << joint_pos.back() << "]\n";
  PRINT_INFO_MSG(oss.str());
}

void MainController::viewStartPose(bool view)
{
  if (view)
  {
    Eigen::Vector3d pos( start_pos(0), start_pos(1), start_pos(2) );
    Eigen::Quaterniond orient( start_quat(0), start_quat(1), start_quat(2), start_quat(3) );
    rviz_pub-> publishFrame(pos, orient, 1.3, "start_pose");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("start_pose");
    rviz_pub->drawnow();
  }
}

void MainController::viewStartJointPos(bool view)
{
  if (view)
  {
    arma::vec start_pos = robot_urdf->getPosition(this->q_start);
    arma::vec start_quat = robot_urdf->getQuat(this->q_start);
    Eigen::Vector3d pos( start_pos(0), start_pos(1), start_pos(2) );
    Eigen::Quaterniond orient( start_quat(0), start_quat(1), start_quat(2), start_quat(3) );
    rviz_pub-> publishFrame(pos, orient, 1.3, "start_joint_pos");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("start_joint_pos");
    rviz_pub->drawnow();
  }
}

ExecResultMsg MainController::replayRecordedMotion()
{
  arma::mat jpos_data = joints_rec_->getJointPosData();
  if (jpos_data.empty()) return ExecResultMsg(ExecResultMsg::WARNING, "There are no recorded data...");

  rw_::Mode prev_mode = robot->getMode(); // store current robot mode
  this->setMode(rw_::JOINT_POS_CONTROL);

  robot->update(); // waits for the next tick
  for (int j=0; j<jpos_data.n_cols; j++)
  {
    robot->setJointsPosition(jpos_data.col(j));
    robot->update(); // waits for the next tick
  }
  this->setMode(prev_mode);

  return ExecResultMsg(ExecResultMsg::INFO, "Finished recorded motion replay!");
}

// ==================================================
// ========  On-robot camera callibration   =========
// ==================================================

#include <tf2_ros/static_transform_broadcaster.h>

void MainController::onRobotCameraCallibrationCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg)
{
  for (auto it = msg->detections.begin(); it!=msg->detections.end(); it++)
  {
    const apriltag_ros::AprilTagDetection &tag = *it;
    if (tag.id[0] == on_robot_tag_id)
    {
      // find camera - tag tf
      auto pose = tag.pose.pose.pose;
      arma::vec pos = {pose.position.x, pose.position.y, pose.position.z};
      arma::vec quat = {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};

      if (!tag_n) prev_quat = quat;
      if (arma::dot(quat, prev_quat) < 0)
      {
        quat = -quat;
        prev_quat = quat;
      }

      tag_pos += pos;
      tag_quat += quat;
      tag_n++;
      robot_cam_tf_sem.notify();
      return;
    }
  }
}

void MainController::onRobotCameraCallibration()
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  ros::Subscriber tag_sub = ros::NodeHandle().subscribe(tag_detections_topic, 1, &MainController::onRobotCameraCallibrationCallback, this);

  tag_n = 0;
  int tag_n_total = 10;
  tag_pos = arma::vec().zeros(3);
  tag_quat = arma::vec().zeros(4);

  ros::AsyncSpinner ros_async_spinner(1);

  ros_async_spinner.start();

  while (tag_n < tag_n_total)
  {
    if (!robot_cam_tf_sem.wait_for(200))
    {
      emit gui->showMsgSignal(ExecResultMsg(ExecResultMsg::WARNING, "Timeout on waiting to update robot-camera transform..."));
      return;
    }
  }
  
  tag_pos /= tag_n_total;
  tag_quat /= tag_n_total;
  tag_quat = tag_quat / arma::norm(tag_quat);

  arma::mat T_cam_tag = arma::mat().eye(4, 4);
  T_cam_tag.submat(0, 3, 2, 3) = tag_pos;
  T_cam_tag.submat(0, 0, 2, 2) = math_::quat2rotm(tag_quat);

  // find robot - on_robot_tag_link tf
  robo_::KinematicChain chain = robo_::KinematicChain();
  chain.initFromParam(this->robot_desc_param, this->base_link, this->robot_tag_link);
  arma::mat T_base_tag = chain.getPose(robot->getJointsPosition().subvec(0, 4));

  // calc the robot-camera tf
  this->T_robot_cam = T_base_tag * arma::inv(T_cam_tag);
  is_T_robot_cam_updated = true;

  // std::cerr << "T_base_tag = \n" << T_base_tag << "\n";
  // std::cerr << "T_tag_cam = \n" << arma::inv(T_cam_tag) << "\n";
  // std::cerr << "T_cam_tag = \n" << T_cam_tag << "\n";
  std::cerr << "=================================\n";
  std::cerr << "T_robot_cam = \n" << T_robot_cam << "\n";

  arma::vec pos = T_robot_cam.submat(0, 3, 2, 3);
  arma::vec quat = math_::rotm2quat(T_robot_cam.submat(0, 0, 2, 2));
  std::cerr << "pos: " << vec2str(pos, 8) << "\n";
  std::cerr << "orient: " << vec2str(quat, 8) << "\n";
  std::cerr << "=================================\n";

  if (!publish_robot_cam_tf_topic.empty())
  {
    ros::Publisher tf_pub = node.advertise<geometry_msgs::TransformStamped>(publish_robot_cam_tf_topic, 1);
    geometry_msgs::TransformStamped Tf_stamp;
    Tf_stamp.header.frame_id = this->base_link;
    Tf_stamp.header.stamp = ros::Time::now();
    Tf_stamp.child_frame_id = camera_tf.camera_opt_frame_link;
    Tf_stamp.transform.translation.x = pos(0);
    Tf_stamp.transform.translation.y = pos(1);
    Tf_stamp.transform.translation.z = pos(2);
    Tf_stamp.transform.rotation.w = quat(0);
    Tf_stamp.transform.rotation.x = quat(1);
    Tf_stamp.transform.rotation.y = quat(2);
    Tf_stamp.transform.rotation.z = quat(3);
    tf_pub.publish(Tf_stamp);
  }

  // std::cerr << "==================================\n";

  // geometry_msgs::TransformStamped static_transformStamped;
  // static_transformStamped.header.stamp = ros::Time::now();
  // static_transformStamped.header.frame_id = tag_on_robot_link;
  // static_transformStamped.child_frame_id = "on_robot_tag";
  // static_transformStamped.transform.translation.x = p_rlink_tag(0);
  // static_transformStamped.transform.translation.y = p_rlink_tag(1);
  // static_transformStamped.transform.translation.z = p_rlink_tag(2);
  // static_transformStamped.transform.rotation.x = Q_rlink_tag(1);
  // static_transformStamped.transform.rotation.y = Q_rlink_tag(2);
  // static_transformStamped.transform.rotation.z = Q_rlink_tag(3);
  // static_transformStamped.transform.rotation.w = Q_rlink_tag(0);
  // static_broadcaster.sendTransform(static_transformStamped);

  updateCameraPoseInRviz(T_robot_cam, this->base_link, this->camera_base_link);

  emit gui->showMsgSignal(ExecResultMsg(ExecResultMsg::INFO, "Robot-cam tf updated successfully!"));
}

void MainController::updateCameraPoseInRviz(const arma::mat &Tf_base_camOpt, const std::string &base, const std::string &cam_base)
{
  tf::Transform T_base_camOpt;
  T_base_camOpt.setOrigin( tf::Vector3( Tf_base_camOpt(0,3), Tf_base_camOpt(1,3), Tf_base_camOpt(2,3) ) );
  T_base_camOpt.setBasis( tf::Matrix3x3( Tf_base_camOpt(0,0), Tf_base_camOpt(0,1), Tf_base_camOpt(0,2), \
                                         Tf_base_camOpt(1,0), Tf_base_camOpt(1,1), Tf_base_camOpt(1,2), \
                                         Tf_base_camOpt(2,0), Tf_base_camOpt(2,1), Tf_base_camOpt(2,2) ) );
  
  // from urdf of realsense
  tf::StampedTransform T_camBase_camOpt;
  T_camBase_camOpt.setOrigin( tf::Vector3(0.0106, 0.0325, 0.0125) );
  T_camBase_camOpt.setRotation( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) );

  tf::Transform T_base_camBase = T_base_camOpt * T_camBase_camOpt.inverse();
  tf::TransformBroadcaster().sendTransform(tf::StampedTransform(T_base_camBase, ros::Time::now(), base, cam_base));
}
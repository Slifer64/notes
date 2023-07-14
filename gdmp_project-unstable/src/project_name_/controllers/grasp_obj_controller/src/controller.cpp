#include <grasp_obj_controller/controller.h>

#include <iostream>
#include <io_lib/xml_parser.h>
#include <io_lib/file_io.h>
#include <math_lib/math_lib.h>
#include <gmp_lib/io/gmp_io.h>
#include <gmp_lib/CanonicalSystem/CanonicalSystem.h>
// #include <matlab_lib/mat_file_writer.h>

#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

using namespace as64_;

namespace fs = boost::filesystem;

#define GraspObjController_fun_ std::string("[GraspObjController::") + __func__ + "]: "

GraspObjController::GraspObjController(MainController *main_ctrl, const std::string &ctrl_name)
: Controller(main_ctrl, ctrl_name), async_spinner(2)
{
  // Gb_::train_ctrl = this;

  async_spinner.start();

  this->main_ctrl = main_ctrl;
  this->robot = main_ctrl->robot.get();

  ros::NodeHandle nh("~");

  default_data_path = ros::package::getPath("grasp_obj_controller") + "/";

  // load params
  ExecResultMsg msg = loadParams();
  if (msg.getType() != ExecResultMsg::INFO) throw std::runtime_error(GraspObjController_fun_ + msg.getMsg());

  // setup rviz
  rviz_pub.reset( new rviz_::RvizMarkerPublisher("/" + getNodeNameID() + "_grasp_obj_ctrl_marker_topic", base_link) );

  // subscribe to rgb image topic
  img_reader.reset(new ros_lib_::ImageReader(params.image_topic, params.depth_image_topic));

  // subscribe to robot_cam tf topic
  robot_cam_tf.reset(new ros_lib_::TransformReader(params.robot_cam_tf_topic));

  // read camera info
  ros_lib_::CameraParams cam_params;
  if (!cam_params.readFromTopic(params.cam_info_topic, 10000))
    throw std::runtime_error(GraspObjController_fun_ + "Timeout on waiting to read camera info from topic'");
  auto cam_info = cam_params.getInfo();
  fx = cam_info.K[0];
  fy = cam_info.K[4];
  cx = cam_info.K[2];
  cy = cam_info.K[5];

  action_client.reset(new actionlib::SimpleActionClient<grasp_obj_controller::ImageToMPAction>(params.resnet_action_name));
}

QPushButton *GraspObjController::createGui(MainWindow *parent)
{
  QPushButton *btn = new QPushButton(this->ctrl_name.c_str());
  GraspObjGui *gui = new GraspObjGui(this, parent);
  QObject::connect( btn, &QPushButton::pressed, parent, [gui](){ gui->launch(); });
  return btn;
}

GraspObjController::~GraspObjController()
{}

ExecResultMsg GraspObjController::loadParams(const std::string &path)
{
  std::string path_ = path;
  try
  {
    if (path_.empty()) path_ = default_data_path + "/config/params.yaml";

    ros::NodeHandle nh("~");
    if (!nh.getParam("base_link", base_link)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param \"base_link\"...");
    if (!nh.getParam("image_topic", params.image_topic)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'image_topic'");
    if (!nh.getParam("depth_image_topic", params.depth_image_topic))
    {
      PRINT_WARNING_MSG("params.depth_image_topic set to ''\n");
      params.depth_image_topic = "";
    }

    if (!nh.getParam("cam_info_topic", params.cam_info_topic)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'cam_info_topic'");
    if (!nh.getParam("robot_cam_tf_topic", params.robot_cam_tf_topic)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'robot_cam_tf_topic'");
    
    // io_::XmlParser parser(default_data_path + "/config/params.yaml");
    YAML::Node node = YAML::LoadFile(path_);
    if (!YAML::getParam(node, "task_n", params.task_n)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'task_n'");
    if (!YAML::getParam(node, "task_p0", params.task_p0)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'task_p0'");

    if (!YAML::getParam(node, "resnet_action_name", params.resnet_action_name)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'resnet_action_name'");

    if (!YAML::getParam(node, "retract_z_offset", params.retract_z_offset)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'retract_z_offset'");
    if (!YAML::getParam(node, "Tf_per_meter", params.Tf_per_meter)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'Tf_per_meter'");
    

    YAML::Node gripper_node;
    if (!YAML::getParam(node, "gripper", gripper_node)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'gripper_node'");
    if (!YAML::getParam(gripper_node, "open_angle", gripper.open_angle)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'gripper_node.open_angle'");
    if (!YAML::getParam(gripper_node, "close_angle", gripper.close_angle)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'gripper_node.close_angle'");
    if (!YAML::getParam(gripper_node, "max_force", gripper.max_force)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'gripper_node.max_force'");

    YAML::Node ph_stop_node;
    if (!YAML::getParam(node, "phase_stop", ph_stop_node)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'ph_stop_node'");
    if (!YAML::getParam(ph_stop_node, "active", phase_stop.active)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'ph_stop_node.active'");
    if (!YAML::getParam(ph_stop_node, "force_scale", phase_stop.force_scale)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'ph_stop_node.force_scale'");
    if (!YAML::getParam(ph_stop_node, "gain", phase_stop.gain)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'ph_stop_node.gain'");

    YAML::Node fp_node;
    if (!YAML::getParam(node, "force_params", fp_node)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'force_params'");
    if (!YAML::getParam(fp_node, "filt", force_params.filt)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'force_params.filt'");
    if (!YAML::getParam(fp_node, "deadzone", force_params.deadzone)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'force_params.deadzone'");
    if (!YAML::getParam(fp_node, "enabled_dofs", force_params.enabled_dofs)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'force_params.enabled_dofs'");

    YAML::Node dmp_params_node;
    if (!YAML::getParam(node, "dmp_params", dmp_params_node)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'dmp_params'");
    if (!YAML::getParam(dmp_params_node, "K", dmp_params.K)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'dmp_params.K'");
    if (!YAML::getParam(dmp_params_node, "M", dmp_params.M)) throw std::runtime_error(GraspObjController_fun_ + "Failed to load param 'dmp_params.M'");
    if (!YAML::getParam(dmp_params_node, "D", dmp_params.D))
    {
      dmp_params.D.resize(dmp_params.K.size());
      for (int i=0; i<dmp_params.D.size(); i++) dmp_params.D[i] = 2*std::sqrt(dmp_params.M[i]*dmp_params.K[i] + 1);
    }

    main_ctrl->gripper_ctrl->setDefaults(gripper.open_angle, gripper.close_angle, gripper.max_force);

    return ExecResultMsg(ExecResultMsg::INFO, "Params loaded successfully!");
  }
  catch (std::exception &e)
  {
    return ExecResultMsg(ExecResultMsg::ERROR, GraspObjController_fun_ + e.what());
  }
  
}

// ===============================================================

void GraspObjController::imageToMP_doneCb(const actionlib::SimpleClientGoalState& state, const grasp_obj_controller::ImageToMPResultConstPtr& result)
{

  if (!result->success)
  {
    std::cerr << "\33[1;31m" + result->msg + "\33[0m\n";
    emit gui->showMsgSignal(ExecResultMsg(ExecResultMsg::ERROR, result->msg));
    return;
  }

  // get robot-cam tf
  if (!robot_cam_tf->isUpdated())
  {
    emit gui->showMsgSignal(ExecResultMsg(ExecResultMsg::ERROR, GraspObjController_fun_ + "robot_cam_tf has not been updated..."));
    return;
  }
  arma::vec p_rc = robot_cam_tf->pos();
  arma::vec Q_rc = robot_cam_tf->quat();
  arma::mat R_rc = math_::quat2rotm(Q_rc);

  // get the mp weights
  arma::mat mp_weights = arma::join_vert(arma::rowvec(result->mp_x_weights), arma::rowvec(result->mp_y_weights), arma::rowvec(result->mp_theta_weights));
  unsigned N_kernels = mp_weights.n_cols;
  gmp_::GMP mp_2d(3, N_kernels);
  mp_2d.W = mp_weights;

  // generate data
  int n_points = 200;
  arma::rowvec s_data = arma::linspace<arma::rowvec>(0, 1, n_points);
  arma::mat y_data(3, n_points);
  for (int j=0; j<n_points; j++) y_data.col(j) = mp_2d.getYd(s_data(j));

  //  calculate the 3D orientation
  arma::mat Q_data(4, n_points);
  for (int j=0; j<n_points; j++)
  {
    double theta = y_data(2, j);
    Q_data.col(j) = math_::axang2quat(arma::join_vert(params.task_n, arma::vec({theta})));
  }

  // calculate the 3D position

  // project xy-data to 3D with unit depth
  arma::mat p1_data(3, n_points);
  p1_data.row(0) = (y_data.row(0)*img_width - cx) / fx;
  p1_data.row(1) = (y_data.row(1)*img_height - cy) / fy;
  p1_data.row(2) = arma::rowvec().ones(n_points); // use unit z

  // Convert task plane normal and point to camera frame
  arma::vec task_n = R_rc.t() * params.task_n;
  arma::vec task_p0 = R_rc.t() * (params.task_p0 - p_rc);

  // calculate the depth and scale accordingly the 3D projected trajectory
  arma::rowvec z = arma::dot(task_n, task_p0) / (task_n.t() * p1_data);
  arma::mat P_data = p1_data % arma::repmat(z, 3, 1);

  // convert the data from camera to robot frame
  P_data = R_rc*P_data + arma::repmat(p_rc, 1, n_points);

  // learn 3D position and orientation data
  mp.reset(new gmp_::GMP(3, N_kernels));
  mp_o.reset(new gmp_::GMPo(N_kernels));

  // std::shared_ptr<gmp_::TrajScale> traj_sc(new gmp_::TrajScale_None(3));
  std::shared_ptr<gmp_::TrajScale> pos_scale(new gmp_::TrajScale_Rot_min());
  mp->setScaleMethod(pos_scale);
  std::shared_ptr<gmp_::TrajScale> orient_scale(new gmp_::TrajScale_Rot_min());
  mp_o->setScaleMethod(orient_scale);

  arma::vec pos_mse, orient_mse;
  mp->train("LS", s_data, P_data, &pos_mse);
  mp_o->train("LS", s_data, Q_data, &orient_mse);

  P0 = mp->getYd(0);
  Q0 = mp_o->getQd(0);
  Pg = mp->getYd(1);
  Qg = mp_o->getQd(1);

  // rviz_pub->publishOrientedPath(P_data, Q_data, 6, rviz_::Color::MAGENTA, 0.02, 1.2, "learned_path");
  // rviz_pub->drawnow();

  std::cerr << "train pos mse: " << pos_mse.t() << "\n";
  std::cerr << "train orient mse: " << orient_mse.t() << "\n";

  record_.mp_weights = mp_weights;
  record_.mp = mp.get();
  record_.mp_o = mp_o.get();

  emit gui->showMsgSignal(ExecResultMsg(ExecResultMsg::INFO, "Received model params!"));
}

ExecResultMsg GraspObjController::record()
{
  std::string err_msg;
  std::string save_path = getDefaultPath() + "/log/";
  bool success = record_.save(save_path, &err_msg);
  if (!success) return ExecResultMsg(ExecResultMsg::ERROR, err_msg);
  return ExecResultMsg(ExecResultMsg::INFO, "Recorded image and mp!");
}

void GraspObjController::imageToMP_activeCb()
{
  PRINT_INFO_MSG("ImageToMP action is active!");
}

void GraspObjController::imageToMP_feedbackCb(const grasp_obj_controller::ImageToMPFeedbackConstPtr& feedback)
{
  // still running...
}

void GraspObjController::readImage()
{
  if (!img_reader->readNewFrame(500))
    throw std::runtime_error(GraspObjController_fun_ + "Timeout on wait to read image...");
  cv::Mat img = img_reader->getRGB();
  img_height = img.rows;
  img_width = img.cols;
  cv::Mat depth_im = img_reader->getDepth();

  record_.img = img;
  record_.depth = depth_im;

  if (!robot_cam_tf->isUpdated()) emit gui->showMsgSignal(ExecResultMsg(ExecResultMsg::WARNING, GraspObjController_fun_ + "robot_cam_tf has not been updated..."));
  // robot_cam_tf_data.push_back(arma::join_vert(robot_cam_tf->pos(), robot_cam_tf->quat()));

  grasp_obj_controller::ImageToMPGoal goal;
  goal.rgb = *cv_bridge::CvImage(std_msgs::Header(), "rgb8", img).toImageMsg();
  goal.model_name = gui->getModelName();
  goal.target_obj_id = gui->getTargetObjId();

  action_client->sendGoal(goal,
              boost::bind(&GraspObjController::imageToMP_doneCb, this, _1, _2),
              boost::bind(&GraspObjController::imageToMP_activeCb, this),
              boost::bind(&GraspObjController::imageToMP_feedbackCb, this, _1));

  // PRINT_INFO_MSG(GraspObjController_fun_ + "Captured image!\n");
}

ExecResultMsg GraspObjController::moveToStartPose()
{
  ExecResultMsg msg;

  // arma::vec P = robot->getTaskPosition();
  // if (arma::norm(P, P0) > 0.2)
  // {
    msg = main_ctrl->gotoStartPose(false);
    if (msg.getType() != ExecResultMsg::INFO) return msg;
  // }

  msg = main_ctrl->moveToCartPose(P0, Q0);
  if (msg.getType() != ExecResultMsg::INFO) return msg;

  return ExecResultMsg(ExecResultMsg::INFO, "Reached start pose!");
}

void GraspObjController::startExec()
{
  s = 0.0;
  
  std::cerr << "\33[1;33mMoving to start pose...\n\33[0m" << std::endl;
  ExecResultMsg msg = moveToStartPose();
  if (msg.getType() != ExecResultMsg::INFO)
  {
    emit gui->stopExecSignal(msg);
    return;
  }

  runModel();
  
  if (gui->auto_record()) record();

  exec_stop_sem.notify();
}

ExecResultMsg GraspObjController::stopExec()
{
  exec_on.set(false);
  this->setMode(rw_::IDLE);
  if ( exec_stop_sem.wait_for(1500) ) return ExecResultMsg(ExecResultMsg::INFO, "Execution stopped!");
  else return ExecResultMsg(ExecResultMsg::WARNING, "Time-out reached on waiting for execution to stop...");
}

void GraspObjController::retract()
{
  ExecResultMsg msg;

  // during retraction, raise a bit up along z, to avoid friction with the surface
  arma::vec P_f = robot->getTaskPosition() + arma::vec({0, 0, params.retract_z_offset});
  msg = main_ctrl->moveToCartPose(P_f, robot->getTaskOrientation());
  if (msg.getType() != ExecResultMsg::INFO) emit gui->stopExecSignal(msg);

  // s = s_current;

  runModel(true); // reverse:=true

  msg = main_ctrl->gotoStartPose(false);
  if (msg.getType() != ExecResultMsg::INFO) emit gui->showMsgSignal(msg);

  // gui->trigger_startExecAction();
}

arma::vec deadZone(const arma::vec &F_ext, const arma::vec &Fext_dead_zone)
{
  arma::vec sign_Fext = arma::sign(F_ext);
  arma::vec Fext2 = F_ext - sign_Fext%Fext_dead_zone;
  return 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);
}


void GraspObjController::runModel(bool reverse)
{
  // if (main_ctrl->robot->getMode() != rw_::Mode::FREEDRIVE) main_ctrl->setMode(rw_::Mode::FREEDRIVE);

  exec_on.set(true);

  this->setMode(rw_::CART_VEL_CTRL);
  robot->update();

  double Ts = robot->getCtrlCycle();

  double t = 0;
  arma::vec P = robot->getTaskPosition();
  arma::vec Q = robot->getTaskOrientation();
  arma::vec Q_prev = Q;

  arma::vec P_init = P0;
  arma::vec Q_init = Q0;
  arma::vec Pf = Pg;
  arma::vec Qf = Qg;

  int dir = 1;
  double sf = 1.0;
  if (reverse)
  {
    dir = -1;
    sf = 0.0;

    arma::vec p_offset = {0, 0, params.retract_z_offset};
    Pf = P0 + p_offset;
    Qf = Q0;

    mp->setY0(Pf);
    mp_o->setQ0(Qf);
    mp->setGoal(P);
    mp_o->setQg(Q);
  }
  else
  {
    mp->setY0(P);
    mp_o->setQ0(Q);
    mp->setGoal(Pf);
    mp_o->setQg(Qf);
  }

  if (arma::norm(P - mp->getYd(s)) > 8e-3 || 
      arma::norm(math_::quatLog(math_::quatDiff(Q, mp_o->getQd(s)))) > 2e-2)
  {
    if (reverse)
    {
      emit gui->stopExecSignal(ExecResultMsg(ExecResultMsg::WARNING, "Aborting execution due to DANGER OF LARGE ACCELERATIONS: The current pose of the robot is far from the reference."));
      return;
    }
  }

  bool reached_target = false;

  double K = dmp_params.K[0];
  double D = dmp_params.D[0];
  double M = dmp_params.M[0];

  double Ko = dmp_params.K[1];
  double Do = dmp_params.D[1];
  double Mo = dmp_params.M[1];

  arma::vec P_dot = arma::vec().zeros(3);
  arma::vec P_ddot = arma::vec().zeros(3);

  arma::vec vRot = arma::vec().zeros(3);
  arma::vec vRot_dot = arma::vec().zeros(3);

  arma::vec Fext = arma::vec().zeros(6);

  // calc executed path length and scale Tf accordingly...
  double exec_len = getExecPathLength();
  double Tf = std::max(params.Tf_per_meter*exec_len, 2.0);
  // double s = this->s;
  double sd_dot = dir / Tf;
  double s_ddot = 0;

  gmp_::CanonicalSystem can_sys(Tf);
  can_sys.s = this->s;
  can_sys.sd_dot = sd_dot;
  // can_sys.s_dot = sd_dot;

  double ep = 1e6;
  double eo = 1e6;

  while (exec_on())
  {
    if (!robot->isOk())
    {
      // showErrorMsg("The robot is not ok...\n Aborting velocity teaching...");
      emit gui->stopExecSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting execution..."));
      break;
    }

    if (main_ctrl->getMode() != rw_::CART_VEL_CTRL)
    {
      // showErrorMsg("The robot is not ok...\n Aborting velocity teaching...");
      emit gui->stopExecSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot mode has changed...\n Aborting execution..."));
      break;
    }

    // external force ???

    arma::vec Fext_new = deadZone(robot->getTaskWrench(), force_params.deadzone);
    Fext = (1 - force_params.filt)*Fext + force_params.filt*Fext_new;
    Fext = Fext % force_params.enabled_dofs;
    
    if (phase_stop.active) can_sys.sd_dot = sd_dot / (1 + phase_stop.gain * arma::norm(Fext.subvec(0, 2)));

    s_ddot = can_sys.getPhaseDDot();

    arma::vec Pd = mp->getYd(can_sys.s);
    arma::vec Pd_dot = mp->getYdDot(can_sys.s, can_sys.s_dot);
    arma::vec Pd_ddot = mp->getYdDDot(can_sys.s, can_sys.s_dot, s_ddot);

    arma::vec Qd = mp_o->getQd(can_sys.s);
    arma::vec vRotd = mp_o->getVd(can_sys.s, can_sys.s_dot);
    arma::vec vRotd_dot = mp_o->getVdDot(can_sys.s, can_sys.s_dot, s_ddot);

    P_ddot = Pd_ddot + D*(Pd_dot - P_dot) + K*(Pd - P) + Fext.subvec(0, 2)/M;
    vRot_dot = vRotd_dot + Do*(vRotd - vRot) + Ko*math_::quatLog(math_::quatDiff(Qd, Q)) + Fext.subvec(3, 5)/Mo;

    // ====================================

    arma::vec V_cmd = arma::join_vert(P_dot, vRot);
    robot->setTaskVelocity(V_cmd, P, Q);

    robot->update();
    can_sys.integrate(t, t+Ts);
    this->s = can_sys.s;
    t = t + Ts;
    P = P + P_dot*Ts;
    P_dot = P_dot + P_ddot*Ts;
    Q = math_::quatProd(math_::quatExp(vRot*Ts), Q);
    vRot = vRot + vRot_dot*Ts;
    // if (arma::dot(Q, Q_prev)<0) Q = -Q;

    ep = arma::norm(Pf - P);
    eo = arma::norm(math_::quatLog(math_::quatDiff(Qf, Q)));
    if (ep < 1e-3 && eo < 5e-3)
    {
      reached_target = true;
      break;
    }

    if (dir*s > sf && t > Tf+2)
    {
      std::cerr << "\33[1;33mTime limit exceeded... pos_err = " << ep << " , orient_err = " << eo << "\n\33[0m" << std::endl;
    }
  }

  robot->setTaskVelocity(arma::vec().zeros(6));
  this->setMode(rw_::IDLE);

  if (!exec_on()) return;

  if (reached_target)
  {
    if (reverse) main_ctrl->gripper_ctrl->open();
    else main_ctrl->gripper_ctrl->close();
    
    if (!main_ctrl->gripper_ctrl->wait(5000))
    {
      emit gui->stopExecSignal(ExecResultMsg(ExecResultMsg::ERROR, "Timeout on waiting for gripper to close..."));
      return;
    }
  }

  if (reached_target) emit gui->stopExecSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished execution!"));
  else emit gui->stopExecSignal(ExecResultMsg(ExecResultMsg::ERROR, "Failed to reach target pose..."));
}

double GraspObjController::getExecPathLength(unsigned n_points) const
{
  arma::rowvec s_data = arma::linspace<arma::rowvec>(0, 1, n_points);
  double len = 0;
  arma::vec P = mp->getYd(s_data(0));
  for (int j=1; j<n_points; j++)
  {
    arma::vec P_next = mp->getYd(s_data(j));
    len += norm(P_next - P);
    P = P_next;
  }
  return len;
}


void GraspObjController::viewExecPath(bool view)
{
  if (view)
  {
    int n_data = 100;
    arma::rowvec s_data = arma::linspace<arma::rowvec>(0, 1, n_data);

    arma::mat Pos(3, n_data);
    for (int j=0; j < n_data; j++) Pos.col(j) = mp->getYd(s_data(j));

    arma::mat Quat = arma::mat().zeros(4, n_data);
    for (int j=0; j<n_data; j++) Quat.col(j) = mp_o->getQd(s_data(j));
    
    rviz_pub->publishOrientedPath(Pos, Quat, 6, rviz_::Color::MAGENTA, 0.02, 1.2, "learned_path");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("learned_path");
    rviz_pub->drawnow();
  }
}


// ==============  Record_  ================

std::string zfill(const std::string &s, int fill, char fill_char='0')
{
  std::string s_fill(s);
  fill = fill - s.length();
  if (fill > 0) s_fill = std::string(fill, fill_char) + s_fill;
  return s_fill;
}


#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

bool GraspObjController::Record_::save(const std::string &path, std::string *err_msg)
{
  std::string folder_name = path + "/" + zfill(std::to_string(sample_count), 5) + "/";
  try
  {
    if (fs::exists(folder_name)) fs::remove_all(folder_name);
    if (!fs::create_directory(folder_name)) throw std::runtime_error("Failed to create directory '" + folder_name + "'...\n");

    cv::Mat bgr_img;
    cv::cvtColor(this->img, bgr_img, cv::COLOR_RGB2BGR);
    if (!cv::imwrite(folder_name + "/rgb.png", bgr_img))
    {
      throw std::runtime_error("Failed to save 'rgb.png'...\n");
    }
    if (!depth.empty() && !cv::imwrite(folder_name + "/depth.png", this->depth))
    {
       throw std::runtime_error("Failed to save 'depth.png'...\n");
    }

    if (!this->mp_weights.save(folder_name + "/mp_weights.txt", arma::raw_ascii)) throw std::runtime_error("Error saving 'mp_weights.txt'...\n");

    gmp_::FileIO fid(folder_name + "/cartesian_mp.bin", io_::FileIO::out | io_::FileIO::trunc);
    gmp_::write(this->mp, fid, "mp_pos");
    gmp_::write(this->mp_o, fid, "mp_orient");

    sample_count++;
    return true;
  }
  catch (std::exception &e)
  {
    if (err_msg) *err_msg = std::string("[GraspObjController::Record_::save]: ") + e.what();
    return false;
  }
}
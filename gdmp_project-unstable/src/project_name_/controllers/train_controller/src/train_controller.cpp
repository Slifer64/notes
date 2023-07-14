#include <train_controller/train_controller.h>

#include <iostream>
#include <io_lib/xml_parser.h>
#include <io_lib/file_io.h>
#include <plot_lib/qt_plot.h>
#include <math_lib/math_lib.h>

#include <ros/package.h>

using namespace as64_;

#define TrainController_fun_ std::string("[TrainController::") + __func__ + "]: "

TrainController::TrainController(MainController *main_ctrl, const std::string &ctrl_name)
: Controller(main_ctrl, ctrl_name)
{
  // Gb_::train_ctrl = this;

  this->main_ctrl = main_ctrl;
  this->robot = main_ctrl->robot.get();

  model.reset(new Model());
  //if (model()) this->target_pose = arma::join_vert(model->getTrainFinalPos(), model->getTrainFinalQuat());
  //emit gui->setTargetPoseSignal(this->target_pose);

  target_pose = {0,0,0, 1,0,0,0};

  ros::NodeHandle nh("~");

  if (!nh.getParam("train_data_path", default_train_data_path)) default_train_data_path = "/config/";
  default_train_data_path = ros::package::getPath("train_controller") + "/" + default_train_data_path;

  std::string train_data_filename;
  if (!nh.getParam("train_data_filename", train_data_filename)) PRINT_WARNING_MSG("Failed to load train data...\n");
  else loadTrainData(getDefaultPath() + train_data_filename);

  if (!nh.getParam("base_link", base_link)) throw std::runtime_error(TrainController_fun_ + "Failed to load param \"base_link\"...");
  rviz_pub.reset( new rviz_::RvizMarkerPublisher("/" + getNodeNameID() + "_train_ctrl_marker_topic", base_link) );

  ref_pose_pub.reset(new RefPosePublisher(base_link, "Ref_pose") );

  ros_sub = node.subscribe("train_controller_ros_actions", 1, &TrainController::rosSubCallback, this);
  // std::thread( [](){ while (ros::ok()) ros::spinOnce(); } ).detach();
}

QPushButton *TrainController::createGui(MainWindow *parent)
{
  QPushButton *btn = new QPushButton(this->ctrl_name.c_str());
  TrainWin *gui = new TrainWin(this, parent);
  QObject::connect( btn, &QPushButton::pressed, parent, [gui](){ gui->launch(); });
  return btn;
}

void TrainController::rosSubCallback(const std_msgs::Int16::ConstPtr& msg)
{
  switch (static_cast<TrainController::RosAction>(msg->data))
  {
    case TrainController::EXEC_TRAJ:
      gui->trigger_executeTrainTrajectoryAction();
      break;
    case TrainController::EXEC_REVESE_TRAJ:
      gui->trigger_executeReverseTrainTrajectoryAction();
      break;
    case TrainController::START_TRAIN:
      gui->trigger_startTrainAction();
      break;
    case TrainController::STOP_TRAIN:
      gui->trigger_stopTrainAction();
      break;
    default:
      PRINT_WARNING_MSG(TrainController_fun_ + "Unrecognized input action...\n");
  }
}

TrainController::~TrainController()
{}

// ===============================================================

void TrainController::setPathTeachMethod(enum PathTeachMethod method)
{
  if (method == PathTeach_FREEDRIVE) main_ctrl->setMode(rw_::Mode::FREEDRIVE);
  else if (method == PathTeach_ADMITTANCE) main_ctrl->setMode(rw_::Mode::ADMITTANCE);
}

void TrainController::trainPhaseChanged(TrainPhase phase)
{
  if (phase == Train_Phase1) setPathTeachMethod(gui->getPathTeachMethod());
  else if (phase == Train_Phase2) main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);
  else if (phase == Train_Phase3) main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);
}

void TrainController::setModelType(const std::string &model_type)
{
  Model::Type type;
  if (model_type.compare("std")==0) type = Model::Type::STD;
  else if (model_type.compare("target")==0) type = Model::Type::TARGET;

  model.reset(type);
}

void TrainController::setStartPoseFromTrainData()
{
  if (demo_data.empty())
  {
    PRINT_WARNING_MSG(TrainController_fun_ + "The training data are empty...\n");
    return;
  }

  main_ctrl->setStartPoseSignal(demo_data.getJointPos(0));
}

void TrainController::setTargetPoseFromModel()
{
  if ( !model->isTrained() )
  {
    PRINT_WARNING_MSG(TrainController_fun_ + "The model is not trained...\n");
    return;
  }

  this->setTargetPose( arma::join_vert(model->getTrainFinalPos(), model->getTrainFinalQuat()) );
}

void TrainController::startDemoRec()
{
  TrainPhase train_phase = gui->getTeachingPhase();

  //demo_data.clear();

  if (train_phase == Train_Phase1) pathTeaching();
  else if (train_phase == Train_Phase2) velocityTeaching();
  else if (train_phase == Train_Phase3) adaptLearnedTrajectory();
  else if (train_phase == Train_Phase4) executeLearnedTrajectory(gui->phase4.tau, gui->phase4.reverse);
  else throw std::runtime_error(TrainController_fun_ + ": <Exception>: Unsupported teaching phase...");
  exec_stop_sem.notify();
}

ExecResultMsg TrainController::stopDemoRec()
{
  exec_on.set(false);
  if ( exec_stop_sem.wait_for(1500) ) return ExecResultMsg(ExecResultMsg::INFO, "Data recording stopped!");
  else return ExecResultMsg(ExecResultMsg::WARNING, "Time-out reached on waiting for stop data recording...");
}

void TrainController::pathTeaching()
{
  // if (main_ctrl->robot->getMode() != rw_::Mode::FREEDRIVE) main_ctrl->setMode(rw_::Mode::FREEDRIVE);

  exec_on.set(true);

  robot->update();

  double Ts = robot->getCtrlCycle();

  double t = 0;
  arma::vec P = robot->getTaskPosition();
  arma::vec Q = robot->getTaskOrientation();
  arma::vec Q_prev = Q;
  arma::vec joint_pos = robot->getJointsPosition();

  arma::vec q_start = robot->getJointsPosition();
  std::cerr << TrainController_fun_ + "q_start = " << q_start.t() << "\n";

  bool new_logging = false;
  if (enable_logging())
  {
    clearDemoData();
    new_logging = true;
  }

  while (exec_on())
  {
    if (!robot->isOk())
    {
      // showErrorMsg("The robot is not ok...\n Aborting velocity teaching...");
      emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting path teaching..."));
      break;
    }

    if (enable_logging()) demo_data.add(t,P,Q,joint_pos);
    // ====================================

    robot->update();
    t = t + Ts;
    P = robot->getTaskPosition();
    Q_prev = Q;
    Q = robot->getTaskOrientation();

    if (arma::dot(Q, Q_prev)<0) Q = -Q;

    joint_pos = robot->getJointsPosition();
  }

  if (new_logging) autoTrimDemoData();
}

void TrainController::velocityTeaching()
{
  x_progress = 0;

  main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);

  if (!isModelTrained())
  {
    emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::WARNING, "The model is not trained...\n Aborting velocity teaching..."));
    return;
  }

  // if (main_ctrl->robot->getMode() != rw_::Mode::CART_VEL_CTRL) main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);

  exec_on.set(true);

  VelTeachMethod vel_teach_meth = gui->getVelTeachMethod();

  bool new_logging = false;
  if (enable_logging())
  {
    clearDemoData();
    new_logging = true;
  }

  robot->update();
  arma::vec P0 = robot->getTaskPosition();
  arma::vec Pg = target_pose.subvec(0,2); // model->getTrainFinalPos();
  arma::vec Q0 = robot->getTaskOrientation();
  arma::vec Qg = target_pose.subvec(3,6); // model->getTrainFinalQuat();


  double dt = robot->getCtrlCycle();

  // ============  Init Position  ===============
  arma::vec P = P0;
  arma::vec dP = arma::vec().zeros(3);
  arma::vec ddP = arma::vec().zeros(3);

  // ============  Init Orient  ===============
  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(3);
  arma::vec dvRot = arma::vec().zeros(3);

  arma::vec q = model->quatToOrient(Q, Q0);
  arma::vec q_dot = arma::vec().zeros(3);
  arma::vec q_ddot = arma::vec().zeros(3);

  arma::vec F_ext = arma::vec().zeros(6);
  arma::vec V_cmd(6);

  double t = 0;
  double x = 0;
  // double xd_dot = 1/Tf;
  double x_dot = 0;
  double x_ddot = 0;
  double fv = 0;

  model->setInitialPose(P0, Q0);
  model->setTargetPose(Pg, Qg);

  double x_incr = 1e-3;
  arma::vec motion_dir;
  double x_temp = x;
  do{
    // motion_dir = model->getRefVel(x_temp, 1);
    motion_dir = model->getRefPos(x_temp + x_incr) - model->getRefPos(x_temp);
    x_temp += x_incr;
  } while (arma::norm(motion_dir) < 1e-3);
  motion_dir /= arma::norm(motion_dir);
  // motion_dir = motion_dir / (arma::norm(motion_dir) + 1e-30);

  int count = 0;

  fext_filt.reset();

  while (exec_on())
  {
    if (!robot->isOk())
    {
      emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting velocity teaching..."));
      main_ctrl->sendEmergencyStopSignal();
      break;
    }

    // ========= Update robot ===========
    robot->update();

    F_ext = robot->getTaskWrench();

    if (enable_logging()) demo_data.add(t, P, Q, robot->getJointsPosition());

    double fv_new;
    if (vel_teach_meth == VelTeach_SLIDER) fv_new = gui->getSliderValue();
    else if (vel_teach_meth == VelTeach_pHRI)
    {
      // arma::vec nd = model->getMotionDir(x);
      // arma::vec nd = model->getRefVel(x, 1);
      // if (arma::norm(nd) > 5e-3) motion_dir =  nd / arma::norm(nd);
      arma::vec new_motion_dir;
      x_temp = x;
      do{
        new_motion_dir = model->getRefPos(x_temp + x_incr) - model->getRefPos(x_temp);
        x_temp += x_incr; 
      } while (arma::norm(new_motion_dir) < 1e-3 && x_temp <1.0);
      if (arma::norm(new_motion_dir) >= 1e-3) motion_dir = new_motion_dir / arma::norm(new_motion_dir);
      

      fv_new = arma::dot(motion_dir, F_ext.subvec(0,2));
    }

    fv = (1-a_filt())*fv + a_filt()*fv_new;
    double xd_dot = 0;
    if (phase_ff()) xd_dot = gui->getPhaseFF() / (1 + 20*arma::norm(fext_filt(F_ext)));
    x_ddot = -Dx()*(x_dot - xd_dot) + a_fv()*fv;

    if (x<=0 && x_ddot < 0) x_ddot = 0;
    if (x>=1 && x_ddot > 0) x_ddot = 0;

    // ========= GMP simulation ===========
    ddP = model->calcAccel(P, dP, x, x_dot, x_ddot);
    q_ddot = model->calcOrientAccel(q, q_dot, x, x_dot, x_ddot);

    // ========= Command robot ===========
    V_cmd.subvec(0,2) = dP ;
    V_cmd.subvec(3,5) = vRot;
    robot->setTaskVelocity(V_cmd, P, Q);
    // robot->setTaskVelocity(V_cmd);

    // ========  Numerical integration  ========
    t = t + dt;
    x = x + x_dot*dt;
    x_dot = x_dot + x_ddot*dt;

    if (x > 1) { x = 1; x_dot = x_ddot = 0; }
    if (x < 0) { x = 0; x_dot = x_ddot = 0; }

    x_progress = x;
    ref_pose_pub->setRefPose(model->getRefPos(x), model->getRefQuat(x));

    P = P + dP*dt;
    dP = dP + ddP*dt;

//    Q = math_::quatProd(math_::quatExp(vRot*dt), Q);
//    vRot = vRot + dvRot*dt;
    q = q + q_dot*dt;
    q_dot = q_dot + q_ddot*dt;

    Q = model->orientToQuat(q, Q0);
    vRot = model->orientVelToRotVel(q_dot, Q, Q0);
  }

  robot->update();
  robot->setTaskVelocity(arma::vec().zeros(6));

  if (new_logging) autoTrimDemoData();
}

void TrainController::adaptLearnedTrajectory()
{
  x_progress = 0;

  main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);

  if (!isModelTrained())
  {
    emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::WARNING, "The model is not trained...\n Aborting velocity teaching..."));
    return;
  }

  // if (main_ctrl->robot->getMode() != rw_::Mode::CART_VEL_CTRL) main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);

  exec_on.set(true);

  bool new_logging = false;
  if (enable_logging())
  {
    clearDemoData();
    new_logging = true;
  }

  robot->update();
  arma::vec P0 = robot->getTaskPosition();
  arma::vec Pg = target_pose.subvec(0,2); // model->getTrainFinalPos();
  arma::vec Q0 = robot->getTaskOrientation();
  arma::vec Qg = target_pose.subvec(3,6); // model->getTrainFinalQuat();


  double dt = robot->getCtrlCycle();

  // ============  Init Position  ===============
  arma::vec P = P0;
  arma::vec dP = arma::vec().zeros(3);
  arma::vec ddP = arma::vec().zeros(3);

  // ============  Init Orient  ===============
  arma::vec Q = Q0;
  arma::vec vRot = arma::vec().zeros(3);
  arma::vec dvRot = arma::vec().zeros(3);

  arma::vec q = model->quatToOrient(Q, Q0);
  arma::vec q_dot = arma::vec().zeros(3);
  arma::vec q_ddot = arma::vec().zeros(3);

  arma::vec F_ext = arma::vec().zeros(6);
  arma::vec V_cmd(6);

  double t = 0;
  double x = 0;
  double x_dot = 0;
  double x_ddot = 0;
  double fv = 0;

  model->setInitialPose(P0, Q0);
  model->setTargetPose(Pg, Qg);

  // arma::vec motion_dir = model->getRefVel(x, 1);
  // motion_dir = motion_dir / (arma::norm(motion_dir) + 1e-30);
  double x_incr = 1e-3;
  arma::vec motion_dir;
  double x_temp = x;
  do{
    // motion_dir = model->getRefVel(x_temp, 1);
    motion_dir = model->getRefPos(x_temp + x_incr) - model->getRefPos(x_temp);
    x_temp += x_incr;
  } while (arma::norm(motion_dir) < 1e-3);
  motion_dir /= arma::norm(motion_dir);
  // motion_dir = motion_dir / (arma::norm(motion_dir) + 1e-30);

  arma::vec orient_motion_dir = model->getRefOrientDot(x, 1);
  orient_motion_dir = orient_motion_dir / (arma::norm(orient_motion_dir) + 1e-30);

  model->initUpdate(); // init anyway in case it is used
  // new_model = mode.deepCopy();

  int count = 0;

  int count2 = 0;

  struct LogData
  {
    arma::vec p;
    arma::vec p_dot;
    arma::vec p_ddot;
    arma::vec pd;
    arma::vec pd_dot;
    arma::vec pd_ddot;
    arma::vec Fext;
    double x;
    double x_dot;
    double x_ddot;
    double fv;
    double f_new;
  };

  LogData log_;
  std::vector<LogData> log_data;
  log_data.reserve(100000);

  // arma::mat P_data;
  // arma::mat P_dot_data;
  // arma::mat P_ddot_data;
  // arma::mat Pd_data;
  // arma::mat Pd_dot_data;
  // arma::mat Pd_ddot_data;
  // arma::mat Fext_data;
  // arma::mat Fc_data;
  // arma::mat spring_force_data;
  // arma::mat phase_var_data; // {x, x_dot, fv}
  // arma::rowvec Kp_data;

  // ==========================================
  // =========  Control loop start  ===========
  // ==========================================
  double af = 0.005;
  double ep_thres = 5e-3;
  double wp_update_thres = 1e-3;

  arma::rowvec x_train_data = model->getXtrainData();
  std::list<double> x_data_list(x_train_data.begin(), x_train_data.end());
  auto x_iter = x_data_list.begin();

  while (exec_on())
  {
    // =========  Check if the robot is ok  ===========
    if (!robot->isOk())
    {
      emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting velocity teaching..."));
      main_ctrl->sendEmergencyStopSignal();
      break;
    }

    // =========  Update robot  ===========
    robot->update();

    if (enable_logging()) demo_data.add(t, P, Q, robot->getJointsPosition());

    // ===========  Get external wrench  ==============
    F_ext = (1-af)*F_ext + af*robot->getTaskWrench();

    if (enable_logging()) demo_data.add(t, P, Q, robot->getJointsPosition());

    // ===========  Find the motion direction ==============

    // Cartesian position motion direction
    // arma::vec nd = model->getRefVel(x, 1);
    // if (arma::norm(nd) > 2e-3) motion_dir =  nd / arma::norm(nd);
    // //else ''keep previous direction''
    arma::vec new_motion_dir;
      x_temp = x;
      do{
        new_motion_dir = model->getRefPos(x_temp + x_incr) - model->getRefPos(x_temp);
        x_temp += x_incr; 
      } while (arma::norm(new_motion_dir) < 1e-3 && x_temp <1.0);
      if (arma::norm(new_motion_dir) >= 1e-3) motion_dir = new_motion_dir / arma::norm(new_motion_dir);

    // Cartesian orientation motion direction
    arma::vec nd = model->getRefOrientDot(x, 1);
    if (arma::norm(nd) > 2e-3) orient_motion_dir =  nd / arma::norm(nd);
    //else ''keep previous direction''

    // ==========  Calc phase variable update based on the force along the motion direction

    double f_new = arma::dot(motion_dir, F_ext.subvec(0,2));
    fv = (1-a_filt())*fv + a_filt()*f_new; // force along the motion direction

    double xd_dot = 0;
    if (phase_ff()) xd_dot = gui->getPhaseFF() / (1 + 20*arma::norm(fext_filt(F_ext)));
    x_ddot = -Dx()*(x_dot - xd_dot) + a_fv()*fv;

    // saturate (just in case)
    if (x<=0 && x_ddot < 0) x_ddot = 0;
    if (x>=1 && x_ddot > 0) x_ddot = 0;

    // ==========  Cartesian position controller  ==========
    // Get reference trajectory
    arma::vec Pd = model->getRefPos(x);
    arma::vec Pd_dot = model->getRefVel(x, x_dot);
    arma::vec Pd_ddot = model->getRefAccel(x, x_dot, x_ddot);
    arma::vec ep = P - Pd;
    // Calc the force that is perpendicular to the motion direction
    arma::vec Fp_c = F_ext.subvec(0,2) - arma::dot(motion_dir, F_ext.subvec(0,2))*motion_dir;
    // impedance params
    double Dp = 20;
    double Kp = 50 + 300*std::exp(-50*arma::norm(ep));
    // calc spring force
    arma::vec spring_force = -Kp*(ep);
    if ( arma::norm(spring_force) > 5) spring_force = 5*spring_force / arma::norm(spring_force);
    // Cartesian position dynamics equation
    ddP = Pd_ddot - Dp*(dP - Pd_dot) + spring_force + 0.4*Fp_c;

    log_.p = P;
    log_.p_dot = dP;
    log_.p_ddot = ddP;
    log_.pd = Pd;
    log_.pd_dot = Pd_dot;
    log_.pd_ddot = Pd_ddot;
    log_.Fext = F_ext;
    log_.f_new = f_new;
    log_.fv = fv;
    log_.x = x;
    log_.x_dot = x_dot;
    log_.x_ddot = x_ddot;
    log_data.push_back(log_);

    // ==========  Cartesian orientation controller  ==========
    // Get reference trajectory (in quat-log space)
    arma::vec qd = model->getRefOrient(x);
    arma::vec qd_dot = model->getRefOrientDot(x, x_dot);
    arma::vec qd_ddot = model->getRefOrientDDot(x, x_dot, x_ddot);
    arma::vec eo = q - qd;
    // Transform the torque in the quat-log space
    arma::vec Q1 = math_::quatDiff(Q,Q0);
    arma::vec Fo_qlog =  0.5 * math_::jacob_qLog_Q(Q1) * math_::quatProd( arma::vec({0, F_ext(3), F_ext(4), F_ext(5) }), Q1 );
    // Calc the log-torque that is perpendicular to the orient (qlog) motion direction
    arma::vec Fo_c = Fo_qlog; // - arma::dot(orient_motion_dir, Fo_qlog)*orient_motion_dir;
    // impedance params
    double Do = 8;
    double Ko = 20 + 100*std::exp(-30*arma::norm(eo));
    arma::vec spring_torque = -Ko*(eo);
    if ( arma::norm(spring_torque) > 6) spring_torque = 6*spring_torque / arma::norm(spring_torque);
    // Cartesian orientation dynamics equation
    q_ddot = qd_ddot - Do*(q_dot - qd_dot) + spring_torque + 6*Fo_c;
    // q_ddot = model->calcOrientAccel(q, q_dot, x, x_dot, x_ddot);

    // sync x_iter with x
    while (*x_iter < x) x_iter++;
    x_iter--;
  
    if (enable_online_adaptation())
    {
      if (arma::norm(ep) > ep_thres)
      {
        model->updatePosition(x, P);
      }
      // if (arma::norm(eo) > 1e-2) model->updateqLogQuat(x, q);

      if (arma::norm(ep) < wp_update_thres) model->updatePositionParams();
      // if (arma::norm(eo) < 1e-3) model->updateOrientationParams();
    }

    
    // if (++count2 == 300)
    // {
    //   //std::cerr << "Vertical error: " << arma::dot(P-Pd, nd) << "\n";
    //   std::cerr << "===================================\n";
    //   // std::cerr << "spring_force: " << arma::norm(spring_force) << "\n";
    //   std::cerr << "spring_torque: " << arma::norm(spring_torque) << "\n";
    //   std::cerr << "|| Fo_c ||: " << arma::norm(Fo_c) << "\n";
    //   std::cerr << "===================================\n";
    //   count2 = 0;
    // }

    // ========= Command robot ===========
    V_cmd.subvec(0,2) = dP;
    V_cmd.subvec(3,5) = vRot;
    robot->setTaskVelocity(V_cmd, P, Q); // use CLICK
    // robot->setTaskVelocity(V_cmd);

    // ===========  Set task progress and reference pose (for gui/rviz)  ============
    x_progress = x;
    ref_pose_pub->setRefPose(Pd, model->getRefQuat(x));


    // if (enable_logging())
    // {
    //   // ===========  log data  ============
    //   P_data = arma::join_horiz(P_data, P);
    //   P_dot_data = arma::join_horiz(P_dot_data, dP);
    //   P_ddot_data = arma::join_horiz(P_ddot_data, ddP);
    //   // -----------------------------------
    //   Pd_data = arma::join_horiz(Pd_data, Pd);
    //   Pd_dot_data = arma::join_horiz(Pd_dot_data, Pd_dot);
    //   Pd_ddot_data = arma::join_horiz(Pd_ddot_data, Pd_ddot);
    //   // -----------------------------------
    //   Fext_data = arma::join_horiz(Fext_data, F_ext.subvec(0,2));
    //   Fc_data = arma::join_horiz(Fc_data, Fp_c);
    //   // -----------------------------------
    //   spring_force_data = arma::join_horiz(spring_force_data, spring_force);
    //   // -----------------------------------
    //   phase_var_data = arma::join_horiz(phase_var_data, arma::vec({x, x_dot, x_ddot, fv}) );
    //   // -----------------------------------
    //   Kp_data = arma::join_horiz(Kp_data, arma::vec({Kp}));
    // }
    

    // ========  Numerical integration  ========
    t = t + dt;
    x = x + x_dot*dt;
    x_dot = x_dot + x_ddot*dt;

    if (x > 1) { x = 1; x_dot = x_ddot = 0; }
    if (x < 0) { x = 0; x_dot = x_ddot = 0; }

    P = P + dP*dt;
    dP = dP + ddP*dt;

    // Q = math_::quatProd(math_::quatExp(vRot*dt), Q);
    // vRot = vRot + dvRot*dt;
    q = q + q_dot*dt;
    q_dot = q_dot + q_ddot*dt;
    Q = model->orientToQuat(q, Q0);
    vRot = model->orientVelToRotVel(q_dot, Q, Q0);
  }

  // ==========================================
  // ==========  Control loop end  ============
  // ==========================================


  robot->update();
  robot->setTaskVelocity(arma::vec().zeros(6)); // to make sure that the robot has indeed stopped

  try
  {
    int n_data = log_data.size();
    arma::mat p_data(3, n_data);
    arma::mat p_dot_data(3, n_data);
    arma::mat p_ddot_data(3, n_data);
    arma::mat pd_data(3, n_data);
    arma::mat pd_dot_data(3, n_data);
    arma::mat pd_ddot_data(3, n_data);
    arma::mat Fext_data(6, n_data);
    arma::rowvec x_data(n_data);
    arma::rowvec x_dot_data(n_data);
    arma::rowvec x_ddot_data(n_data);
    arma::rowvec fv_data(n_data);
    arma::rowvec f_new_data(n_data);

    for (int j=0; j<n_data; j++)
    {
      p_data.col(j) = log_data[j].p;
      p_dot_data.col(j) = log_data[j].p_dot;
      p_ddot_data.col(j) = log_data[j].p_ddot;
      pd_data.col(j) = log_data[j].pd;
      pd_dot_data.col(j) = log_data[j].pd_dot;
      pd_ddot_data.col(j) = log_data[j].pd_ddot;
      Fext_data.col(j) = log_data[j].Fext;
      x_data(j) = log_data[j].x;
      x_dot_data(j) = log_data[j].x_dot;
      x_ddot_data(j) = log_data[j].x_ddot;
      fv_data(j) = log_data[j].fv;
      f_new_data(j) = log_data[j].f_new;
    }

    io_::FileIO fid(this->default_train_data_path + "phase3_data.bin", io_::FileIO::out | io_::FileIO::trunc);
    fid.write("p_data", p_data);
    fid.write("p_dot_data", p_dot_data);
    fid.write("p_ddot_data", p_ddot_data);
    fid.write("pd_data", pd_data);
    fid.write("pd_dot_data", pd_dot_data);
    fid.write("pd_ddot_data", pd_ddot_data);
    fid.write("Fext_data", Fext_data);
    fid.write("x_data", x_data);
    fid.write("x_dot_data", x_dot_data);
    fid.write("x_ddot_data", x_ddot_data);
    fid.write("fv_data", fv_data);
    fid.write("f_new_data", f_new_data);
    fid.close();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  // if (!P_data.empty())
  // { 
  //   io_::FileIO fid(this->default_train_data_path + "phase3_data.bin", io_::FileIO::out | io_::FileIO::trunc);
  //   // -----------------------------------
  //   fid.write("P_data", P_data);
  //   fid.write("P_dot_data", P_dot_data);
  //   fid.write("P_ddot_data", P_ddot_data);
  //   // -----------------------------------
  //   fid.write("Pd_data", Pd_data);
  //   fid.write("Pd_dot_data", Pd_dot_data);
  //   fid.write("Pd_ddot_data", Pd_ddot_data);
  //   // -----------------------------------
  //   fid.write("Fext_data", Fext_data);
  //   fid.write("Fc_data", Fc_data);
  //   // -----------------------------------
  //   fid.write("spring_force_data", spring_force_data);
  //   // -----------------------------------
  //   fid.write("phase_var_data", phase_var_data);
  //   // -----------------------------------
  //   fid.write("Kp_data", Kp_data);
  //   // -----------------------------------
  //   fid.close();
  // }

  if (new_logging) autoTrimDemoData();
}

void TrainController::executeLearnedTrajectory(double tau, bool reverse)
{
  if (!isModelTrained())
  {
    emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::WARNING, "The model is not trained...\n Aborting execution..."));
    return;
  }

  main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);

  x_progress = 0;

  exec_on.set(true);

  robot->update();
  arma::vec P0 = robot->getTaskPosition();
  arma::vec Pg = target_pose.subvec(0,2); // model->getTrainFinalPos();
  arma::vec Q0 = robot->getTaskOrientation();
  arma::vec Qg = target_pose.subvec(3,6); // model->getTrainFinalQuat();

  arma::vec V_cmd(6);

  double t = 0;
  double dt = robot->getCtrlCycle();
  double x = 0;
  double x_dot = 1 / tau;
  double x_end = 1;

  bool new_logging = false;
  if (enable_logging())
  {
    clearDemoData();
    new_logging = true;
  }

  // if (reverse)
  // {
  //   x = 1;
  //   x_dot = -x_dot;
  //   x_end = 0;
  //   Pg = robot->getTaskPosition();
  //   Qg = robot->getTaskOrientation();
  //   P0 = model->getTrainInitPos();
  //   Q0 = model->getTrainInitQuat();
  // }

  model->setInitialPose(P0, Q0);
  model->setTargetPose(Pg, Qg);

  while (exec_on())
  {
    if (!robot->isOk())
    {
      emit gui->ctrl_execModelFinishedSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting execution..."));
      main_ctrl->sendEmergencyStopSignal();
      break;
    }

    // ========= Update robot ===========
    robot->update();

  
    arma::vec P = model->getRefPos(x);
    arma::vec Q = model->getRefQuat(x);
    if (enable_logging()) demo_data.add(t, P, Q, robot->getJointsPosition());

    // ========= Command robot ===========
    V_cmd.subvec(0,2) = model->getRefVel(x, x_dot) ;
    V_cmd.subvec(3,5) = model->getRefRotVel(x, x_dot) ;
    robot->setTaskVelocity(V_cmd, P, Q);

    // ========  Numerical integration  ========
    t = t + dt;
    x = x + x_dot*dt;

    if (std::fabs(x-x_end) <= 0.001) break;

    x_progress = x;
  }

  robot->update();
  robot->setTaskVelocity(arma::vec().zeros(6));

  // if (!demo_data.empty() && new_logging)
  // {
  //   demo_data.trim(0.001, 0.001);
  //   //demo_data.removeStops(5e-3);
  // }

  emit gui->ctrl_execModelFinishedSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished execution!"));
}

void TrainController::executeTrainTrajectory(bool reverse)
{
  if (!isModelTrained())
  {
    emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::WARNING, "The model is not trained...\n Aborting execution..."));
    return;
  }

  main_ctrl->setMode(rw_::Mode::CART_VEL_CTRL);

  x_progress = 0;

  exec_on.set(true);

  robot->update();
  arma::vec P0 = robot->getTaskPosition();
  arma::vec Pg = target_pose.subvec(0, 2); //model->getTrainFinalPos();
  arma::vec Q0 = robot->getTaskOrientation();
  arma::vec Qg = target_pose.subvec(3,6); //model->getTrainFinalQuat();

  double dt = robot->getCtrlCycle();

  arma::vec V_cmd(6);

  double x = 0;
  double x_dot = 1 / model->getTrainDuration();
  double x_end = 1;

  if (reverse)
  {
    x = 1;
    x_dot = -x_dot;
    x_end = 0;
    Pg = robot->getTaskPosition();
    Qg = robot->getTaskOrientation();
    P0 = P0_exec;
    Q0 = Q0_exec;
  }

  P0_exec = P0;
  Q0_exec = Q0;
  model->setInitialPose(P0, Q0);
  model->setTargetPose(Pg, Qg);

  while (exec_on())
  {
    if (!robot->isOk())
    {
      emit gui->ctrl_execModelFinishedSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting execution..."));
      main_ctrl->sendEmergencyStopSignal();
      break;
    }

    // ========= Update robot ===========
    robot->update();

    // ========= Command robot ===========
    V_cmd.subvec(0,2) = model->getRefVel(x, x_dot) ;
    V_cmd.subvec(3,5) = model->getRefRotVel(x, x_dot) ;
    robot->setTaskVelocity(V_cmd, model->getRefPos(x), model->getRefQuat(x));

    // ========  Numerical integration  ========
    x = x + x_dot*dt;

    if (std::fabs(x-x_end) <= 0.001) break;

    x_progress = x;
  }

  robot->update();
  robot->setTaskVelocity(arma::vec().zeros(6));

  emit gui->ctrl_execModelFinishedSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished execution!"));
}

void TrainController::clearDemoData()
{
  demo_data.clear();
}

ExecResultMsg TrainController::trainModel()
{
  try {

    if (!isTrainData()) return ExecResultMsg(ExecResultMsg::WARNING, "The training data are empty!\nModel training aborted...");

    model->initialize(N_kernels());
    model->train(train_method(), demo_data.getTime(), demo_data.getPos(), demo_data.getQuat());

    P0_exec = model->getTrainInitPos();
    Q0_exec = model->getTrainInitQuat();

    return ExecResultMsg(ExecResultMsg::INFO, "The model has been trained!");
  }
  catch(std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, TrainController_fun_ + "Exception: " + e.what()); }
}

ExecResultMsg TrainController::equalizeVelocityProfile()
{
  try {

    if (!isModelTrained()) return ExecResultMsg(ExecResultMsg::WARNING, "The model is not trained! Aborting velocity profile equalization...");

    unsigned iters = eq_vel_prof_iters();
    double nom_vel = eq_vel_prof_nom_vel();

    // ============ Equalize velocity profile ============
    for (int k=0; k<iters; k++)
    {
      arma::rowvec Time;
      arma::mat P_data, Q_data;

      double dt = 0.001;
      double t = 0;
      double x = 0;
      double x_dot = 0;

      while (x<1)
      {
        double xd_dot = nom_vel / arma::norm( model->getRefVel(x,1) + 1e-16 );
        double x_ddot = 200*(xd_dot - x_dot);

        arma::vec P = model->getRefPos(x);
        arma::vec Q = model->getRefQuat(x);

        Time = arma::join_horiz( Time, arma::vec({t}) );
        P_data = arma::join_horiz( P_data, P );
        Q_data = arma::join_horiz( Q_data, Q );

        t = t + dt;
        x = x + x_dot*dt;
        x_dot = x_dot + x_ddot*dt;
      }

      model->train(train_method(), Time, P_data, Q_data);
    }

    return ExecResultMsg(ExecResultMsg::INFO, "Velocity profile equalization completed!");
  }
  catch(std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, TrainController_fun_ + "Exception: " + e.what()); }
}

ExecResultMsg TrainController::loadTrainData(const std::string &path)
{
  try
  {
    arma::rowvec Timed;
    arma::mat Pd_data, Qd_data, joint_pos_data;

    io_::FileIO fid(path, io_::FileIO::in);
    fid.read("Timed", Timed);
    fid.read("Pd_data", Pd_data);
    fid.read("Qd_data", Qd_data);
    fid.read("joint_pos_data", joint_pos_data);

    demo_data.setData(Timed, Pd_data, Qd_data, joint_pos_data);

    // main_ctrl->setStartPoseSignal(joint_pos_data.col(0));

    PRINT_INFO_MSG(TrainController_fun_ + "The training data have been loaded successfully!\n");
    return ExecResultMsg(ExecResultMsg::INFO, "The training data have been loaded successfully!");
  }
  catch(std::exception &e){
    PRINT_ERROR_MSG(TrainController_fun_ + e.what() + "\n");
    return ExecResultMsg(ExecResultMsg::ERROR, TrainController_fun_ + e.what()); }
}

ExecResultMsg TrainController::saveTrainData(const std::string &path)
{
  try
  {
    if (!isTrainData()) return ExecResultMsg(ExecResultMsg::WARNING, "The training data are empty!\nSave training data aborted...");

    io_::FileIO fid(path, io_::FileIO::out | io_::FileIO::trunc);
    fid.write("Timed", demo_data.getTime());
    fid.write("Pd_data", demo_data.getPos());
    fid.write("Qd_data", demo_data.getQuat());
    fid.write("joint_pos_data", demo_data.getJointPos());
    PRINT_INFO_MSG(TrainController_fun_+ "The training data have been saved successfully!\n");
    return ExecResultMsg(ExecResultMsg::INFO, "The training data have been saved successfully!");
  }
  catch(std::exception &e){
    PRINT_ERROR_MSG(TrainController_fun_ + e.what() + "\n");
    return ExecResultMsg(ExecResultMsg::ERROR, TrainController_fun_ + e.what()); }
}

void TrainController::autoTrimDemoData()
{
  if (!demo_data.empty() && autotrim_train_data())
  {
    double vel_trim_thres = gui->getAutoTrimMoveThreshold();
    demo_data.trim(vel_trim_thres, vel_trim_thres);
    //demo_data.removeStops(5e-3);
  }
}

ExecResultMsg TrainController::trimDemoData(double pos_thres, double orient_thres)
{
  if (!isTrainData()) return ExecResultMsg(ExecResultMsg::ERROR, "The training data are empty...");
  demo_data.trim(pos_thres, orient_thres);
  return ExecResultMsg(ExecResultMsg::INFO, "The training data were trimmed successfully!");
}

ExecResultMsg TrainController::undoTrimDemoData()
{
  if (!isTrainData()) return ExecResultMsg(ExecResultMsg::WARNING, "The training data are empty!");
  demo_data.undoTrim();
  return ExecResultMsg(ExecResultMsg::INFO, "Trimming was successfully undone!");
}

ExecResultMsg TrainController::removeStopsFromDemo(double vel_thres)
{
  if (!isTrainData()) return ExecResultMsg(ExecResultMsg::ERROR, "The training data are empty...");
  demo_data.removeStops(vel_thres);
  return ExecResultMsg(ExecResultMsg::INFO, "Stops were removed from the training data!");
}

void TrainController::plotTrainData()
{
  if (!isTrainData())
  {
    showWarningMsg("The training data are empty!");
    return;
  }

  arma::rowvec Time = demo_data.getTime();
  arma::mat P_data = demo_data.getPos();
  arma::mat dP_data = demo_data.getVel();
  arma::mat Q_data = demo_data.getQuat();
  arma::mat vRot_data = demo_data.getRotVel();


  int n_data = Time.size();
  arma::rowvec vel_norm(n_data);
  for (int j=0; j<n_data; j++) vel_norm(j) = arma::norm(dP_data.col(j));

  pl_::Figure *fig_ = pl_::figure("", {500, 600});
  fig_->setAxes(1,1);
  pl_::Axes *ax = fig_->getAxes(0);
  ax->plot(Time, vel_norm, pl_::LineWidth_,2.0, pl_::Color_,pl_::RED);
  ax->title("Translational Velocity L2 norm", pl_::FontSize_,16, pl_::FontWeight_,pl_::DemiBold);
  ax->xlabel("Time [s]", pl_::FontSize_,14);
  // ax->drawnow();
  // fig_->drawnow();

  // ===========  plot position  ===========
  pl_::Figure *p_fig = pl_::figure("", {500, 600});


  p_fig->setAxes(2,1);

  ax = p_fig->getAxes(0);
  ax->hold(true);
  // ax->xlabel("time [s]");
  for (int i=0;i<3;i++)
  {
    pl_::Graph *graph = ax->plot(Time, P_data.row(i), pl_::LineWidth_,2.0);
//    graph->setLineWidth(1);
//    graph->setLineStyle(DashDotLine);
//    graph->setMarkerStyle(ssStar);
//    graph->setMarkerSize(10);
  }
  ax->title("Cartesian Position", pl_::FontSize_,16, pl_::FontWeight_,pl_::DemiBold);
  ax->ylabel("Position [m]", pl_::FontSize_,14);
  ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
  // ax->drawnow();

  ax = p_fig->getAxes(1);
  ax->hold(true);
  // ax->xlabel("time [s]");
  for (int i=0;i<3;i++) ax->plot(Time, dP_data.row(i), pl_::LineWidth_,2.0);
  // ax->title("Cartesian Velocity");
  ax->ylabel("Velocity [m/s]", pl_::FontSize_,14);
  ax->xlabel("Time [s]", pl_::FontSize_,14);
  ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
  // ax->drawnow();

  // p_fig->drawnow();

  // ===========  plot orientation  ===========
  n_data = Q_data.n_cols;
  arma::mat q_data(3, n_data);
  arma::vec Q0 = Q_data.col(0);
  for (int j=0; j<n_data; j++) q_data.col(j) = model->quatToOrient(Q_data.col(j), Q0);

  pl_::Figure *q_fig = pl_::figure("", {500, 600});
  q_fig->setAxes(2,1);

  ax = q_fig->getAxes(0);
  ax->hold(true);
  // ax->xlabel("time [s]");
  for (int i=0;i<3;i++) ax->plot(Time, q_data.row(i), pl_::LineWidth_,2.0);
  ax->title("Cartesian Orientation", pl_::FontSize_,16, pl_::FontWeight_,pl_::DemiBold);
  ax->ylabel("Quaternion error", pl_::FontSize_,14);
  ax->legend({"eq_x", "eq_y", "eq_z"}, pl_::FontSize_,15);
  // ax->drawnow();

  ax = q_fig->getAxes(1);
  ax->hold(true);
  // ax->xlabel("time [s]");
  for (int i=0;i<3;i++) ax->plot(Time, vRot_data.row(i), pl_::LineWidth_,2.0);
  ax->ylabel("Angular Vel [rad/s]", pl_::FontSize_,14);
  ax->xlabel("Time [s]", pl_::FontSize_,14);
  ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
  // ax->drawnow();

  // q_fig->drawnow();

  pl_::drawnow();
}

void TrainController::plotModelSimData()
{
  if (!isTrainData())
  {
    showWarningMsg("The training data are empty!");
    return;
  }

  if (!isModelTrained())
  {
    showWarningMsg("The model is not trained!");
    return;
  }

  // ========  simulate model  ===========

  double tau = demo_data.getDuration();
  double x = 0;
  double x_dot = 1/tau;
  double dt = demo_data.getTime(1) - demo_data.getTime(0);

  arma::rowvec Time = arma::linspace<arma::rowvec>(0, tau, std::round(tau/dt));
  int n_data = Time.size();
  arma::mat P_data(3, n_data), dP_data(3, n_data);
  arma::mat q_data(3, n_data), vRot_data(3, n_data);

  model->setInitialPose(demo_data.getPos(0), demo_data.getQuat(0));
  model->setTargetPose(demo_data.getPos("end"), demo_data.getQuat("end"));

  for (int j=0; j<n_data; j++)
  {
    x = Time(j)/tau;
    P_data.col(j) = model->getRefPos(x);
    dP_data.col(j) = model->getRefVel(x, x_dot);
    q_data.col(j) = model->getRefOrient(x);
    vRot_data.col(j) = model->getRefRotVel(x, x_dot);
  }

  // ========  Plot  =============
  arma::rowvec Time_demo = demo_data.getTime();
  arma::mat P_demo = demo_data.getPos();
  arma::mat dP_demo = demo_data.getVel();
  arma::mat Q_demo = demo_data.getQuat();
  arma::mat vRot_demo = demo_data.getRotVel();

  int n_demo = Q_demo.n_cols;
  arma::mat q_demo(3, n_demo);
  arma::vec Q0 = Q_demo.col(0);
  for (int j=0; j<n_demo; j++) q_demo.col(j) = gmp_::GMPo::quat2q(Q_demo.col(j), Q0);

  // ===========  plot position  ===========
  pl_::Figure *p_fig = pl_::figure("", {700, 500});
  p_fig->setAxes(2,3);
  pl_::Axes *ax;

  std::vector<std::vector<std::string>> ax_legend = { {"x", "xd"}, {"y", "yd"}, {"z", "zd"} };

  for (int j=0;j<3;j++)
  {
    ax = p_fig->getAxes(0,j);
    ax->hold(true);
    ax->plot(Time_demo, P_demo.row(j), pl_::LineWidth_,4.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_,pl_::GREEN);
    ax->plot(Time, P_data.row(j), pl_::LineWidth_,2.0, pl_::Color_,pl_::BLUE);
    ax->legend(ax_legend[j], pl_::FontSize_,15);
    if (j==0) ax->ylabel("Position [m]", pl_::FontSize_,14);
    ax->drawnow();

    ax = p_fig->getAxes(1,j);
    ax->hold(true);
    ax->plot(Time_demo, dP_demo.row(j), pl_::LineWidth_,4.0, pl_::Color_,pl_::GREEN);
    ax->plot(Time, dP_data.row(j), pl_::LineWidth_,2.0, pl_::Color_,pl_::BLUE);
    if (j==0) ax->ylabel("Velocity [m/s]", pl_::FontSize_,14);
    ax->xlabel("Time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }

  // ===========  plot orientation  ===========
  pl_::Figure *q_fig = pl_::figure("", {700, 500});
  q_fig->setAxes(2,3);

  for (int j=0;j<3;j++)
  {
    ax = q_fig->getAxes(0,j);
    ax->hold(true);
    ax->plot(Time_demo, q_demo.row(j), pl_::LineWidth_,4.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_,pl_::GREEN);
    ax->plot(Time, q_data.row(j), pl_::LineWidth_,2.0, pl_::Color_,pl_::BLUE);
    ax->legend(ax_legend[j], pl_::FontSize_,15);
    if (j==0) ax->ylabel("Quat-log", pl_::FontSize_,14);
    ax->drawnow();

    ax = q_fig->getAxes(1,j);
    ax->hold(true);
    ax->plot(Time_demo, vRot_demo.row(j), pl_::LineWidth_,4.0, pl_::Color_,pl_::GREEN);
    ax->plot(Time, vRot_data.row(j), pl_::LineWidth_,2.0, pl_::Color_,pl_::BLUE);
    ax->legend(ax_legend[j], pl_::FontSize_,15);
    if (j==0) ax->ylabel("Rot-Vel [rad/s]", pl_::FontSize_,14);
    ax->xlabel("Time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }

}

void TrainController::plotVelocityProfile()
{
  if (!isModelTrained())
  {
    showWarningMsg("The model is not trained!");
    return;
  }

  // ========  simulate model  ===========
  double Tf = model->getTrainDuration();
  double x = 0;
  double x_dot = 1/Tf;
  double dt = 0.002;

  arma::rowvec Time = arma::linspace<arma::rowvec>(0, Tf, std::round(Tf/dt));
  int n_data = Time.size();
  arma::rowvec vel_norm(n_data);

  model->setInitialPose(model->getTrainInitPos(), model->getTrainInitQuat());
  model->setTargetPose(model->getTrainFinalPos(), model->getTrainFinalQuat());

  for (int j=0; j<n_data; j++) vel_norm(j) = arma::norm( model->getRefVel(Time(j)/Tf, x_dot) );

  // ===========  plot position velocity norm  ===========
  pl_::Figure *fig_ = pl_::figure("", {700, 500});
  fig_->setAxes(1,1);
  pl_::Axes *ax = fig_->getAxes(0);
  ax->plot(Time/Time.back(), vel_norm, pl_::LineWidth_,4.0, pl_::Color_,pl_::RED);
  ax->ylabel("|| vel || [m/s]", pl_::FontSize_,14);
  ax->xlabel("time [s]", pl_::FontSize_,14);
  ax->drawnow();
}

void TrainController::viewLearnedPath(bool view)
{
  if (view)
  {
    arma::vec P0 = robot->getTaskPosition(); //model->getTrainInitPos();
    arma::vec Pg = target_pose.subvec(0,2); // model->getTrainFinalPos();
    arma::vec Q0 = robot->getTaskOrientation(); //model->getTrainInitQuat();
    arma::vec Qg = target_pose.subvec(3,6); // model->getTrainFinalQuat();
    double duration = 5;

    arma::mat Pos = model->getPositionPath(P0, Pg, Q0, Qg, duration);
    arma::mat Quat = model->getQuatPath(P0, Pg, Q0, Qg, duration);

    rviz_pub->publishOrientedPath(Pos, Quat, 6, rviz_::Color::MAGENTA, 0.02, 1.2, "learned_path");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("learned_path");
    rviz_pub->drawnow();
  }
}

void TrainController::viewTargetPose(bool view)
{
  if (view)
  {
    Eigen::Vector3d pos( target_pose(0), target_pose(1), target_pose(2) );
    Eigen::Quaterniond orient( target_pose(3), target_pose(4), target_pose(5), target_pose(6) );
    rviz_pub-> publishFrame(pos, orient, 1.3, "target_pose");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("target_pose");
    rviz_pub->drawnow();
  }
}

void TrainController::setTargetPose(const arma::vec &target)
{
  target_pose = target;
  double Q_norm = arma::norm(target_pose.subvec(3,6));
  if (Q_norm != 1) target_pose.subvec(3,6) /= Q_norm;
  emit gui->setTargetPoseSignal(target_pose);
}

arma::vec TrainController::getTargetPose() const
{
  return target_pose;
}

void TrainController::setCurrentPoseAsTarget()
{
  setTargetPose( arma::join_vert(robot->getTaskPosition(), robot->getTaskOrientation()) );
}

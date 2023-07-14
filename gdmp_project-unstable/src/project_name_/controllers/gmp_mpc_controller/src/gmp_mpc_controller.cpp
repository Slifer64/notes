#include <gmp_mpc_controller/gmp_mpc_controller.h>

#include <gmp_mpc_controller/experiment/handover_experiment.h>
#include <gmp_mpc_controller/experiment/viapoints_experiment.h>
#include <gmp_mpc_controller/experiment/obstacles_experiment.h>

#include <gmp_lib/io/gmp_io.h>
#include <yaml-cpp/yaml.h>
#include <io_lib/file_io.h>

#include <plot_lib/qt_plot.h>

#include <gmp_lib/CanonicalSystem/CanonicalSystem.h>

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/numeric/odeint.hpp>



// using namespace boost::numeric::odeint;

using namespace as64_;

#define GmpMpcController_fun_ std::string("[GmpMpcController::") + __func__ + "]: "

GmpMpcController::GmpMpcController(MainController *main_ctrl, const std::string &ctrl_name)
: Controller(main_ctrl, ctrl_name)
{
  ctrl_map["uncostr"] = std::bind(&GmpMpcController::runUncostr, this, false);
  ctrl_map["MPC"] = std::bind(&GmpMpcController::runMPC, this, false);
  ctrl_map["offline opt"] = std::bind(&GmpMpcController::runOfflineOpt, this, false);
  ctrl_map["repulsive forces"] = std::bind(&GmpMpcController::runRepForces, this, false);


  resetExpFun_map["handover"] = [this](){ exp.reset(new gmp_mpc_ctrl_::HandoverExperiment(this)); };
  resetExpFun_map["via-points"] = [this](){ exp.reset(new gmp_mpc_ctrl_::ViapointsExperiment(this)); };
  resetExpFun_map["obstacles"] = [this](){ exp.reset(new gmp_mpc_ctrl_::ObstaclesExperiment(this)); };

  this->robot = this->main_ctrl->robot.get();

  ros::NodeHandle nh("~");

  default_config_file = ros::package::getPath("gmp_mpc_controller") + "/config/params.yaml";
  ExecResultMsg result = loadParams(default_config_file);
  if (result.getType() != ExecResultMsg::INFO)
  {
    if (result.getType() == ExecResultMsg::WARNING) PRINT_WARNING_MSG(result.getMsg());
    else if (result.getType() == ExecResultMsg::ERROR)
    {
      PRINT_ERROR_MSG(result.getMsg());
      exit(-1);
    }
  }

  gmp.reset(new gmp_::GMP());
  ExecResultMsg msg = loadModel(getModelPath() + "/model.bin");
  if (msg.getType() != ExecResultMsg::INFO) PRINT_WARNING_MSG(msg.getMsg() + "\n");

  if (!nh.getParam("base_link", base_link)) throw std::runtime_error(GmpMpcController_fun_ + "Failed to load param \"base_link\"...");

  this->marker_array_topic = "/" + getNodeNameID() + "_mpc_ctrl_marker_topic";
  
  this->rviz_pub.reset( new rviz_::RvizMarkerPublisher(this->marker_array_topic, this->base_link) );
}

GmpMpcController::~GmpMpcController()
{}

QPushButton *GmpMpcController::createGui(MainWindow *parent)
{
  QPushButton *btn = new QPushButton(this->ctrl_name.c_str());
  GmpMpcWin *gui = new GmpMpcWin(this, parent);
  QObject::connect( btn, &QPushButton::pressed, parent, [gui](){ gui->launch(); });
  return btn;
}

void GmpMpcController::setCurrentPoseAsTarget()
{
  target_pos = robot->getTaskPosition();
  target_quat = robot->getTaskOrientation();
}

bool GmpMpcController::isModelLoaded() const 
{ 
  return is_model_loaded; 
}

ExecResultMsg GmpMpcController::loadModel(const std::string &path)
{
  try
  {
    gmp_::read(gmp.get(), path);
    is_model_loaded = true;
    return ExecResultMsg(ExecResultMsg::INFO, "Loaded the model successfully");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, GmpMpcController_fun_ + e.what()); }

}

ExecResultMsg GmpMpcController::saveModel(const std::string &path)
{
  try
  {
    gmp_::write(gmp.get(), path);
    return ExecResultMsg(ExecResultMsg::INFO, "Saved the model successfully");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, GmpMpcController_fun_ + e.what()); }
}

ExecResultMsg GmpMpcController::saveLogData(const std::string &path)
{
  try
  {
    if (mpc_data.isEmpty()) throw std::runtime_error("There are no logged data...");

    gmp_::FileIO fid(path, gmp_::FileIO::out | gmp_::FileIO::trunc);

    fid.write("Time", mpc_data.Time);
    fid.write("P_data", mpc_data.P_data);
    fid.write("dP_data", mpc_data.Pdot_data);
    fid.write("ddP_data", mpc_data.Pddot_data);
    fid.write("target_data", mpc_data.target_data);
    fid.write("pos_slack_data", mpc_data.pos_slack_data);
    fid.write("vel_slack_data", mpc_data.vel_slack_data);
    fid.write("accel_slack_data", mpc_data.accel_slack_data);
    fid.write("slack_gains", mpc_cfg.slack_gains);
    return ExecResultMsg(ExecResultMsg::INFO, "Saved logged data successfully");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, GmpMpcController_fun_ + e.what()); }
}

ExecResultMsg GmpMpcController::loadParams(const std::string &path)
{
  try
  {
    YAML::Node config = YAML::LoadFile(path);

    if ( !YAML::getParam(config, "use_CLIK", use_click) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'use_CLIK'...");

    if ( !YAML::getParam(config, "data_path", data_path) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'data_path'...");
    data_path = ros::package::getPath("gmp_mpc_controller") + "/" + data_path;

    if ( !YAML::getParam(config, "config_path", config_path) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'config_path'...");
    config_path = ros::package::getPath("gmp_mpc_controller") + "/" + config_path;

    if ( !YAML::getParam(config, "model_path", model_path) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'model_path'...");
    model_path = ros::package::getPath("gmp_mpc_controller") + "/" + model_path;

    if ( !YAML::getParam(config, "q_start", q_start) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'q_start'...");

    // load kinematic limits
    if ( !YAML::getParam(config, "pos_lim", pos_lim) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'pos_lim'...");
    if ( !YAML::getParam(config, "vel_lim", vel_lim) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'vel_lim'...");
    if (vel_lim.n_rows == 1) vel_lim = arma::repmat(vel_lim, 3, 1);
    if ( !YAML::getParam(config, "accel_lim", accel_lim) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'accel_lim'...");
    if (accel_lim.n_rows == 1) accel_lim = arma::repmat(accel_lim, 3, 1);

    // load target_pose struct
    YAML::Node node;
    if ( !YAML::getParam(config, "target_pose", node) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'target_pose'...");
    if (!node.IsMap()) throw std::runtime_error(GmpMpcController_fun_"'target_pose' must be a struct...");
    if ( !YAML::getParam(node, "pos", target_pos) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'target_pose.pos'...");
    if ( !YAML::getParam(node, "quat", target_quat) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'target_pose.quat'...");
    if ( !YAML::getParam(node, "tau", tau) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'target_pose.tau'...");

    // load MPC settings
    ExecResultMsg mcp_msg = loadMpcParams(path, this->mpc_cfg);
    if (mcp_msg.getType() != ExecResultMsg::INFO) throw std::runtime_error(mcp_msg.getMsg());

    // load rep-force settings
    ExecResultMsg rf_msg = loadRepForceParams(path);
    if (rf_msg.getType() != ExecResultMsg::INFO) throw std::runtime_error(rf_msg.getMsg());

    // load offline optimization settings
    ExecResultMsg off_opt_msg = loadOffLineOptParams(path);
    if (off_opt_msg.getType() != ExecResultMsg::INFO) throw std::runtime_error(off_opt_msg.getMsg());

    // load apriltag via-points
    ExecResultMsg msg = loadViaPointsConfig(path);
    if (msg.getType() != ExecResultMsg::INFO) throw std::runtime_error(msg.getMsg());
    
    return ExecResultMsg(ExecResultMsg::INFO, "Loaded params successfully");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, GmpMpcController_fun_ + e.what()); }
}

void GmpMpcController::changeController(const std::string &ctrl_name)
{
  auto it = ctrl_map.find(ctrl_name);
  if (it == ctrl_map.end()) throw std::runtime_error(GmpMpcController_fun_ + "controller '" + ctrl_name + "' is not implemented...\n");

  run_fun_ptr = it->second;
}

std::vector<std::string> GmpMpcController::controllerTypes() const
{
  std::vector<std::string> ctrl_types;
  for (std::map<std::string, std::function<void()> >::const_iterator it = ctrl_map.begin(); it!=ctrl_map.end(); it++) ctrl_types.push_back(it->first);

  return ctrl_types;
}

void GmpMpcController::changeExperiment(const std::string &exp_name)
{
  auto it = resetExpFun_map.find(exp_name);
  if (it == resetExpFun_map.end()) throw std::runtime_error(GmpMpcController_fun_ + "experiment '" + exp_name + "' is not implemented...\n");
  (it->second)();
}

std::vector<std::string> GmpMpcController::experimentTypes() const
{
  std::vector<std::string> exp_types;
  for (std::map<std::string, std::function<void()> >::const_iterator it = resetExpFun_map.begin(); it!=resetExpFun_map.end(); it++) exp_types.push_back(it->first);

  return exp_types;
}

bool GmpMpcController::isOk() const
{
  // -------- Check for external stop  ---------
  if (!run_on)
  {
    emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::INFO, "Execution stopped..."));
    return false;
  }

  // --------  Check if the robot is ok  --------
  if (!robot->isOk())
  {
    emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok:\n" + robot->getErrMsg() + "\n Aborting execution..."));
    main_ctrl->sendEmergencyStopSignal();
    return false;
  }

  return true;
}

void GmpMpcController::run()
{
  if (!run_fun_ptr) throw std::runtime_error(GmpMpcController_fun_ + "'run' function is unassigned...\n");
  run_on = true;
  run_fun_ptr();
  run_on = false;
  stop_sem.notify();
}

// =========  MPC  =========

ExecResultMsg GmpMpcController::loadMpcParams(const std::string &path, MpcSettings &mpc_cfg)
{
  try
  {
    YAML::Node config = YAML::LoadFile(path);
    // load MPC settings
    YAML::Node node;
    if ( !YAML::getParam(config, "mpc_settings", node) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings'...");
    if (!node.IsMap()) throw std::runtime_error(GmpMpcController_fun_"'mpc_settings' must be a struct...");
    if ( !YAML::getParam(node, "N_horizon", mpc_cfg.N_horizon) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.N_horizon'...");
    if ( !YAML::getParam(node, "pred_time_step", mpc_cfg.pred_time_step) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.pred_time_step'...");
    if ( !YAML::getParam(node, "N_kernels", mpc_cfg.N_kernels) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.N_kernels'...");
    if ( !YAML::getParam(node, "kernels_std_scaling", mpc_cfg.kernels_std_scaling) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.kernels_std_scaling'...");
    if ( !YAML::getParam(node, "kernels_trunc_thres", mpc_cfg.kernels_trunc_thres) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.kernels_trunc_thres'...");
    if ( !YAML::getParam(node, "opt_pos_gain", mpc_cfg.opt_pos_gain) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.opt_pos_gain'...");
    if ( !YAML::getParam(node, "opt_vel_gain", mpc_cfg.opt_vel_gain) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.opt_vel_gain'...");
    if ( !YAML::getParam(node, "slack_gains", mpc_cfg.slack_gains) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.slack_gains'...");
    if ( !YAML::getParam(node, "slack_limits", mpc_cfg.slack_limits) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.slack_limits'...");
    if ( !YAML::getParam(node, "time_limit", mpc_cfg.time_limit) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.time_limit'...");
    if ( !YAML::getParam(node, "max_iter", mpc_cfg.max_iter) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.max_iter'...");
    if ( !YAML::getParam(node, "abs_tol", mpc_cfg.abs_tol) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.abs_tol'...");
    if ( !YAML::getParam(node, "rel_tol", mpc_cfg.rel_tol) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'mpc_settings.rel_tol'...");
    if ( !YAML::getParam(node, "final_state_err_tol", mpc_cfg.final_state_err_tol) )
      mpc_cfg.final_state_err_tol = arma::vec().zeros(3,1);

    return ExecResultMsg(ExecResultMsg::INFO, "Loaded MPC params successfully");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, GmpMpcController_fun_ + e.what()); }
}

void GmpMpcController::initMPC(gmp_::GMP_MPC::Ptr &gmp_mpc)
{
  int N_horizon = mpc_cfg.N_horizon;
  double pred_time_step = mpc_cfg.pred_time_step;
  int N_kernels = mpc_cfg.N_kernels;
  double kernels_std_scaling =  mpc_cfg.kernels_std_scaling;

  double opt_pos_gain = mpc_cfg.opt_pos_gain;
  double opt_vel_gain = mpc_cfg.opt_vel_gain;

  gmp_mpc.reset( new gmp_::GMP_MPC(gmp.get(), N_horizon, pred_time_step, N_kernels, kernels_std_scaling, mpc_cfg.slack_gains, mpc_cfg.kernels_trunc_thres) );

  gmp_mpc->setObjCostGains(opt_pos_gain, opt_vel_gain);

  gmp_mpc->settings.time_limit = mpc_cfg.time_limit;
//  gmp_mpc->settings.max_iter = mpc_cfg.max_iter;
  gmp_mpc->settings.abs_tol = mpc_cfg.abs_tol;
  gmp_mpc->settings.rel_tol = mpc_cfg.rel_tol;

  gmp_mpc->setPosLimits(pos_lim.col(0), pos_lim.col(1));
  gmp_mpc->setVelLimits(vel_lim.col(0), vel_lim.col(1));
  gmp_mpc->setAccelLimits(accel_lim.col(0), accel_lim.col(1));

  gmp_mpc->setPosSlackLimit(mpc_cfg.slack_limits[0]);
  gmp_mpc->setVelSlackLimit(mpc_cfg.slack_limits[1]);
  gmp_mpc->setAccelSlackLimit(mpc_cfg.slack_limits[2]);

//  std::cerr << "N_horizon = " << N_horizon << "\n"
//            << "pred_time_step = " << pred_time_step << "\n"
//            << "N_kernels = " << N_kernels << "\n"
//            << "kernels_std_scaling = " << kernels_std_scaling << "\n"
//            << "pos_slack = " << pos_slack << "\n"
//            << "vel_slack = " << vel_slack << "\n"
//            << "accel_slack = " << accel_slack << "\n"
//            << "slack_gains = " << arma::rowvec(slack_gains) << "\n"
//            << "slack_limits = " << mpc_cfg.slack_limits.t() << "\n"
//            << "time_limit = " << gmp_mpc->settings.time_limit << "\n"
//            << "abs_tol = " << gmp_mpc->settings.abs_tol << "\n"
//            << "rel_tol = " << gmp_mpc->settings.rel_tol << "\n"
//            << "pos_lim = \n" << pos_lim << "\n"
//            << "vel_lim = \n" << vel_lim << "\n"
//            << "accel_lim = \n" << accel_lim << "\n";

}

void GmpMpcController::runMPC(bool sim)
{
  ExecResultMsg result = loadParams(default_config_file);
  if (result.getType() != ExecResultMsg::INFO)
  {
    emit gui->execStoppedSignal(result);
    return;
  }

  robot->update();
  arma::vec y0 = robot->getTaskPosition();
  bool found_target;
  arma::vec yg = getTargetPosition(&found_target);
  if (!found_target)
  {
    emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::WARNING, "Failed to detect the target..."));
    return;
  }

  unsigned n_dof = y0.size();
  arma::vec ones_ndof = arma::vec().ones(n_dof);
  arma::vec O_ndof = arma::vec().zeros(n_dof);

  if (log_data) mpc_data.clear();

  double t = 0;
  double dt = robot->getCtrlCycle(); //0.002;
  double s = t/tau;
  double s_dot = 1/tau;
  double s_ddot = 0;
  arma::vec y = y0;
  arma::vec y_dot = O_ndof;
  arma::vec y_ddot = O_ndof;

  // --------  GMP - MPC  --------
  gmp_::GMP_MPC::Ptr gmp_mpc;
  initMPC(gmp_mpc);

  gmp_mpc->setInitialState(y, y_dot, y_ddot, s, s_dot, s_ddot);
  gmp_mpc->setFinalState(yg, O_ndof, O_ndof, 1, s_dot, 0, mpc_cfg.final_state_err_tol);

  auto can_sys_fun = [](double s, double s_dot){ return std::array<double,2>({s_dot, 0.0}); };
  gmp_mpc->setCanonicalSystemFunction(can_sys_fun);

  // gmp->setScaleMethod( gmp_::TrajScale::Ptr( new gmp_::TrajScale_Prop(n_dof) ) );
  gmp->setY0(y0);
  gmp->setGoal(yg);

  arma::rowvec elaps_t_data;
  arma::wall_clock timer, timer2;
  timer.tic();

  if (!sim) main_ctrl->setMode(rw_::CART_VEL_CTRL);

  if (!sim) robot->update();

  arma::vec y_dot_prev = arma::vec().zeros(n_dof);

  double pos_cost = 0;
  double vel_cost = 0;
  int n_data = 0;

  // =========  Simulation loop  =========
  while (true)
  {
    if ( !sim && !isOk() ) goto end_loop;

    yg = getTargetPosition(&found_target);
    if (!found_target) PRINT_WARNING_MSG("Failed to detect the target... Keeping previous position.\n");

    gmp->setGoal(yg);
    gmp_mpc->setFinalState(yg, O_ndof, O_ndof, 1, s_dot, 0, mpc_cfg.final_state_err_tol);

    // for (ViaPoint& vp : via_points)
    // {
    //   if (vp.)
    // }
    
    // --------  Stopping criteria  --------
    if (s > 1.0) break;
    // if (s>1.0 && arma::norm(y-yg)<5e-3 && arma::norm(y_dot)<1e-2) break;

    // if (s>=1.3)
    // {
    //   PRINT_WARNING_MSG("Time limit exceeded...");
    //   if (!sim) emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::WARNING, "Time limit exceeded..."));
    //   goto end_loop;
    // }

    // if (s >= 1) { s = 1; s_dot = 0; s_ddot = 0; }

    timer2.tic();
    gmp_::GMP_MPC::Solution sol = gmp_mpc->solve(s, s_dot);

    if (sol.exit_flag > 0) PRINT_WARNING_MSG(sol.exit_msg + "\n");
    else if (sol.exit_flag < 0)
    {
      PRINT_ERROR_MSG(sol.exit_msg + "\n");
      if (!sim) emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::ERROR, "Failed to find solution: " + sol.exit_msg));
      goto end_loop;
    }

    y = sol.y;
    y_dot = sol.y_dot;
    y_ddot = sol.y_ddot;
    arma::vec pos_slack_var = sol.pos_slack;
    arma::vec vel_slack_var = sol.vel_slack;
    arma::vec accel_slack_var = sol.accel_slack;
    elaps_t_data = arma::join_horiz( elaps_t_data, arma::vec({timer2.toc()*1000}) );

    if (!sim)
    {
      arma::vec V_cmd = arma::vec().zeros(6);
      V_cmd.subvec(0,2) = y_dot;
      if (use_click) robot->setTaskVelocity(V_cmd, y, robot->getTaskOrientation());
      else robot->setTaskVelocity(V_cmd);
      robot->update();
    }
    
    // --------  Log data  --------
    
    // y = robot->getTaskPosition();
    // y_dot = robot->getTaskVelocity();
    // y_ddot = (y_dot - y_dot_prev) / dt;
    // y_dot_prev = y_dot;

    // gmp_mpc->setInitialState(y, y_dot, y_ddot, s, s_dot, s_ddot);

    pos_cost += arma::norm( y - gmp->getYd(s) );
    vel_cost += arma::norm( y_dot - gmp->getYdDot(s,s_dot) );
    n_data++;

    if (log_data) mpc_data.add(t, y, y_dot, y_ddot, yg, pos_slack_var, vel_slack_var, accel_slack_var);

    // --------  Numerical integration  --------
    t = t + dt;

    std::array<double,2> phase_dot = can_sys_fun(s, s_dot);
    s = s + phase_dot[0]*dt;
    s_dot = s_dot + phase_dot[1]*dt;
  }
  
  pos_cost = std::sqrt( pos_cost / n_data);
  vel_cost = std::sqrt( vel_cost / n_data);

  //robot->setTaskVelocity(arma::vec().zeros(6));

  if (!sim) emit gui->execFinishSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished!"));

  end_loop:

  if (!sim) main_ctrl->setMode(rw_::IDLE);

  double elaps_t_ms = timer.toc()*1000;
  PRINT_INFO_MSG("===> GMP-MPC optimization finished! Elaps time: " + std::to_string(elaps_t_ms) + " ms\n");

  if (!elaps_t_data.empty())
  {
    double mean_elaps_t = arma::mean(elaps_t_data);
    double std_elaps_t = arma::stddev(elaps_t_data);
    double max_elaps_t = arma::max(elaps_t_data);
    double min_elaps_t = arma::min(elaps_t_data);

    std::cerr << "======= Elapsed time (ms) ======\n";
    std::cerr << "std_range: [" << std::max(0.,mean_elaps_t-std_elaps_t) << " -  " << mean_elaps_t + std_elaps_t <<"]\n";
    std::cerr << "mean     : " << mean_elaps_t << " +/- " << std_elaps_t <<"\n";
    std::cerr << "min      : " << min_elaps_t << "\n";
    std::cerr << "max      : " << max_elaps_t << "\n";
    std::cerr << "==============================\n";

    std::cerr << "======= Final state errors (m) ======\n";
    std::cerr << "pos_err = " << arma::norm(yg - y) << "\n";
    std::cerr << "vel_err = " << arma::norm(y_dot) << "\n";
    std::cerr << "accel_err = " << arma::norm(y_ddot) << "\n";
    std::cerr << "==============================\n";

    std::cerr << "======= Cost function (m) ======\n";
    std::cerr << "RMSE_pos = " << pos_cost << "\n";
    std::cerr << "RMSE_vel = " << vel_cost << "\n";
    std::cerr << "==============================\n";
  }
}

void GmpMpcController::viewMpcPath(bool view)
{
  if (view)
  {
    std::thread thr(&GmpMpcController::runMPC, this, true);
    makeThreadRT(thr);
    if (thr.joinable()) thr.join();

    arma::mat Pos = mpc_data.P_data;
    rviz_pub->publishPath(Pos, rviz_::Color::ORANGE, 0.02, "mpc_path");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("mpc_path");
    rviz_pub->drawnow();
  }
}

void GmpMpcController::plotMpcResults()
{
  if (mpc_data.isEmpty())
  {
    showWarningMsg("There are no data to plot...");
    return;
  }

  arma::rowvec& Time             = mpc_data.Time;
  arma::mat&    P_data           = mpc_data.P_data;
  arma::mat&    dP_data          = mpc_data.Pdot_data;
  arma::mat&    ddP_data         = mpc_data.Pddot_data;
  arma::mat&    pos_slack_data   = mpc_data.pos_slack_data;
  arma::mat&    vel_slack_data   = mpc_data.vel_slack_data;
  arma::mat&    accel_slack_data = mpc_data.accel_slack_data;

  // ===========  plot position  ===========
  pl_::Figure *p_fig;
  pl_::Axes *ax;

  std::vector<std::string> titles = { "x coordinate", "y coordinate", "z coordinate" };

  double tf = this->tau;
  arma::rowvec t_lim = { Time(0), tf };
  arma::rowvec t_target = {tf};
  for (int i=0;i<3;i++)
  {
    p_fig = pl_::figure("", {500, 700});
    p_fig->setAxes(3,1);

    ax = p_fig->getAxes(0);
    ax->hold(true);
    ax->plot(Time, P_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {pos_lim(i,0),pos_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {pos_lim(i,1),pos_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({target_pos(i)}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->title(titles[i], pl_::FontSize_,16);
    ax->drawnow();

    ax = p_fig->getAxes(1);
    ax->hold(true);
    ax->plot(Time, dP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {vel_lim(i,0),vel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {vel_lim(i,1),vel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->drawnow();

    ax = p_fig->getAxes(2);
    ax->hold(true);
    ax->plot(Time, ddP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {accel_lim(i,0),accel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {accel_lim(i,1),accel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->xlabel("time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }

  bool pos_slack = mpc_cfg.slack_gains[0] > 0;
  bool vel_slack = mpc_cfg.slack_gains[1] > 0;
  bool accel_slack = mpc_cfg.slack_gains[2] > 0;
  int n_slack = pos_slack + vel_slack + accel_slack;
  if (n_slack)
  {
    int n_dof = P_data.n_rows;
    std::vector<std::string> slack_labels;
    std::vector<double> slack_limits;
    std::vector<arma::mat *> slack_data;
    if (pos_slack)
    {
      slack_limits.push_back(mpc_cfg.slack_limits[0]);
      slack_labels.push_back("pos");
      slack_data.push_back(&pos_slack_data);
    }
    if (vel_slack)
    {
      slack_limits.push_back(mpc_cfg.slack_limits[1]);
      slack_labels.push_back("vel");
      slack_data.push_back(&vel_slack_data);
    }
    if (accel_slack)
    {
      slack_limits.push_back(mpc_cfg.slack_limits[2]);
      slack_labels.push_back("accel");
      slack_data.push_back(&accel_slack_data);
    }

    p_fig = pl_::figure("", {800, 700});
    p_fig->setAxes(n_slack,n_dof);
    for (int j=0; j<n_dof; j++)
    {
      for (int i=0;i<n_slack;i++)
      {
        ax = p_fig->getAxes(i,j);
        ax->hold(true);
        ax->plot(Time, slack_data[i]->row(j), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::MAGENTA);
        // ax->plot(t_lim, {slack_limits[i],slack_limits[i]}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
        // ax->plot(t_lim, {-slack_limits[i],-slack_limits[i]}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
        if (j==0) ax->ylabel(slack_labels[i], pl_::FontSize_,14);
        //if (i==0) ax->title("slack variables", pl_::FontSize_,16);
        if (i==n_slack-1) ax->xlabel("time [s]", pl_::FontSize_,14);
        ax->drawnow();
      }
      
    }
  }
}

// =========  Offline Opt  =========

int GmpMpcController::offLineOpt(gmp_::GMP *gmp, const arma::vec &y0, const arma::vec &yg, double tau, std::string *err_msg)
{
  gmp->setY0(y0);
  gmp->setGoal(yg);

  unsigned n_dof = gmp->numOfDoFs();
  arma::vec ones_ndof = arma::vec().ones(n_dof);

  gmp->setTruncatedKernels(offline_opt_cfg.kernels_trunc_thres);

  gmp_::GMP_Opt gmp_opt(gmp);
  gmp_opt.settings.w_p = offline_opt_cfg.opt_pos_gain;
  gmp_opt.settings.w_v = offline_opt_cfg.opt_vel_gain;
  gmp_opt.settings.max_iter = offline_opt_cfg.max_iter;
  gmp_opt.settings.time_limit = offline_opt_cfg.time_limit;
  gmp_opt.settings.abs_tol = offline_opt_cfg.abs_tol;
  gmp_opt.settings.rel_tol = offline_opt_cfg.rel_tol;
  gmp_opt.settings.warm_start = true;

  gmp_opt.setMotionDuration(tau);

  int n_points = 200;
  arma::rowvec x_ineq = arma::linspace<arma::rowvec>(0,1, n_points);

  arma::mat pos_lb = arma::repmat(pos_lim.col(0), 1, n_points);
  arma::mat pos_ub = arma::repmat(pos_lim.col(1), 1, n_points);

  arma::mat vel_lb = arma::repmat(vel_lim.col(0), 1, n_points);
  arma::mat vel_ub = arma::repmat(vel_lim.col(1), 1, n_points);

  arma::mat accel_lb = arma::repmat(accel_lim.col(0), 1, n_points);
  arma::mat accel_ub = arma::repmat(accel_lim.col(1), 1, n_points);

  arma::rowvec x_eq = {0, 1};
  arma::mat pos_eq = arma::join_horiz(y0, yg);
  arma::mat vel_eq = arma::mat().zeros(3,2);
  arma::mat accel_eq = arma::mat().zeros(3,2);

  gmp_opt.setPosConstr(x_ineq, pos_lb, pos_ub, x_eq, pos_eq);
  gmp_opt.setVelConstr(x_ineq, vel_lb, vel_ub, x_eq, vel_eq);
  gmp_opt.setAccelConstr(x_ineq, accel_lb, accel_ub, x_eq, accel_eq);

  arma::wall_clock timer; timer.tic();
  int status = gmp_opt.optimize( arma::linspace<arma::rowvec>(0,1, 200) );
  std::cout << "Optimization finished: " << timer.toc()*1000 << " ms\n";
  if (status > 0) PRINT_INFO_MSG( gmp_opt.getExitMsg() + "\n" );
  else if (status == 0) PRINT_WARNING_MSG( gmp_opt.getExitMsg() + "\n" );
  else if (status < 0) PRINT_ERROR_MSG( gmp_opt.getExitMsg() + "\n" );

  if (err_msg) *err_msg = gmp_opt.getExitMsg();
  return status;
}

ExecResultMsg GmpMpcController::loadOffLineOptParams(const std::string &path)
{
  try
  {
    YAML::Node config = YAML::LoadFile(path);
    // load MPC settings
    YAML::Node node;
    if ( !YAML::getParam(config, "offline_opt_settings", node) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings'...");
    if (!node.IsMap()) throw std::runtime_error(GmpMpcController_fun_"'offline_opt_settings' must be a struct...");
    if ( !YAML::getParam(node, "opt_pos_gain", offline_opt_cfg.opt_pos_gain) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings.opt_pos_gain'...");
    if ( !YAML::getParam(node, "opt_vel_gain", offline_opt_cfg.opt_vel_gain) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings.opt_vel_gain'...");
    if ( !YAML::getParam(node, "time_limit", offline_opt_cfg.time_limit) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings.time_limit'...");
    if ( !YAML::getParam(node, "max_iter", offline_opt_cfg.max_iter) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings.max_iter'...");
    if ( !YAML::getParam(node, "abs_tol", offline_opt_cfg.abs_tol) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings.abs_tol'...");
    if ( !YAML::getParam(node, "rel_tol", offline_opt_cfg.rel_tol) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings.rel_tol'...");
    if ( !YAML::getParam(node, "kernels_trunc_thres", offline_opt_cfg.kernels_trunc_thres) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'offline_opt_settings.kernels_trunc_thres'...");

    return ExecResultMsg(ExecResultMsg::INFO, "Loaded offline-opt params successfully");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, GmpMpcController_fun_ + e.what()); }
}

void GmpMpcController::runOfflineOpt(bool sim)
{
  ExecResultMsg result = loadParams(default_config_file);
  if (result.getType() != ExecResultMsg::INFO)
  {
    emit gui->execStoppedSignal(result);
    return;
  }

  robot->update();
  arma::vec y0 = robot->getTaskPosition();
  bool found_target;
  arma::vec yg = getTargetPosition(&found_target);
  if (!found_target)
  {
    emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::WARNING, "Failed to detect the target..."));
    return;
  }

  unsigned n_dof = y0.size();
  arma::vec O_ndof = arma::vec().zeros(n_dof);

  if (log_data) offline_opt_data.clear();

  // ------------- Execution ----------------

  double t = 0;
  double dt = robot->getCtrlCycle(); //0.002;
  double s = t/tau;
  double s_dot = 1/tau;
  double s_ddot = 0;
  arma::vec y = y0;
  arma::vec y_dot = O_ndof;
  arma::vec y_ddot = O_ndof;

  double pos_cost = 0;
  double vel_cost = 0;
  int n_data = 0;

  gmp_::GMP gmp2;
  gmp->deepCopy(&gmp2);

  std::string err_msg;
  int status = offLineOpt(&gmp2, y0, yg, tau, &err_msg);
  if (status < 0)
  {
    if (!sim) emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::ERROR, "Optimization failed: "+ err_msg));
    goto end_loop;
  }

  if (!sim)
  {
    main_ctrl->setMode(rw_::CART_VEL_CTRL);
    robot->update();
  }

  // =========  Simulation loop  =========
  while (true)
  {
    if ( !sim && !isOk() ) goto end_loop;

    // --------  Stopping criteria  --------
    if (s > 1.0) break;

    // if (s >= 1) { s = 1; s_dot = 0; s_ddot = 0; }

    y = gmp2.getYd(s);
    y_dot = gmp2.getYdDot(s, s_dot);
    y_ddot = gmp2.getYdDDot(s, s_dot, s_ddot);

    if (!sim)
    {
      arma::vec V_cmd = arma::vec().zeros(6);
      V_cmd.subvec(0,2) = y_dot;
      if (use_click) robot->setTaskVelocity(V_cmd, y, robot->getTaskOrientation());
      else robot->setTaskVelocity(V_cmd);
      robot->update();
    }
    
    pos_cost += arma::norm( y - gmp->getYd(s) );
    vel_cost += arma::norm( y_dot - gmp->getYdDot(s,s_dot) );
    n_data++;

    // --------  Log data  --------
    if (log_data) offline_opt_data.add(t, y, y_dot, y_ddot);

    // --------  Numerical integration  --------
    t = t + dt;
    s = s + s_dot*dt;
    s_dot = s_dot + s_ddot*dt;
  }

  pos_cost = std::sqrt( pos_cost / n_data);
  vel_cost = std::sqrt( vel_cost / n_data);

  //robot->setTaskVelocity(arma::vec().zeros(6));
  if (!sim)
  {
    main_ctrl->setMode(rw_::IDLE);
    emit gui->execFinishSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished!"));
  }

  std::cerr << "======= Final state errors (m) ======\n";
  std::cerr << "pos_err = " << arma::norm(yg - y) << "\n";
  std::cerr << "vel_err = " << arma::norm(y_dot) << "\n";
  std::cerr << "accel_err = " << arma::norm(y_ddot) << "\n";
  std::cerr << "==============================\n";

  std::cerr << "======= Cost function (m) ======\n";
  std::cerr << "RMSE_pos = " << pos_cost << "\n";
  std::cerr << "RMSE_vel = " << vel_cost << "\n";
  std::cerr << "==============================\n";

  end_loop:

  return;
}

void GmpMpcController::viewOfflineOptPath(bool view)
{
  if (view)
  {
    runOfflineOpt(true);
    arma::mat Pos = offline_opt_data.P_data;
    rviz_pub->publishPath(Pos, rviz_::Color::GREEN, 0.02, "offline_opt_path");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("offline_opt_path");
    rviz_pub->drawnow();
  }
}

void GmpMpcController::plotOffLineOptResults()
{
  if (offline_opt_data.isEmpty())
  {
    showWarningMsg("There are no data to plot...");
    return;
  }

  arma::rowvec& Time       = offline_opt_data.Time;
  arma::mat&    P_data     = offline_opt_data.P_data;
  arma::mat&    dP_data    = offline_opt_data.Pdot_data;
  arma::mat&    ddP_data   = offline_opt_data.Pddot_data;

  // ===========  plot position  ===========
  pl_::Figure *p_fig;
  pl_::Axes *ax;

  std::vector<std::string> titles = { "x coordinate", "y coordinate", "z coordinate" };

  double tf = this->tau;
  arma::rowvec t_lim = { Time(0), tf };
  arma::rowvec t_target = {tf};
  for (int i=0;i<3;i++)
  {
    p_fig = pl_::figure("", {500, 700});
    p_fig->setAxes(3,1);

    ax = p_fig->getAxes(0);
    ax->hold(true);
    ax->plot(Time, P_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {pos_lim(i,0),pos_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {pos_lim(i,1),pos_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({target_pos(i)}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->title(titles[i], pl_::FontSize_,16);
    ax->drawnow();

    ax = p_fig->getAxes(1);
    ax->hold(true);
    ax->plot(Time, dP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {vel_lim(i,0),vel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {vel_lim(i,1),vel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->drawnow();

    ax = p_fig->getAxes(2);
    ax->hold(true);
    ax->plot(Time, ddP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {accel_lim(i,0),accel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {accel_lim(i,1),accel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->xlabel("time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }
}

// =========  Unconstrained  =========

void GmpMpcController::runUncostr(bool sim)
{
  ExecResultMsg result = loadParams(default_config_file);
  if (result.getType() != ExecResultMsg::INFO)
  {
    emit gui->execStoppedSignal(result);
    return;
  }

  if (!sim) main_ctrl->setMode(rw_::CART_VEL_CTRL);

  robot->update();
  arma::vec y0 = robot->getTaskPosition();
  bool found_target;
  arma::vec yg = getTargetPosition(&found_target);
  if (!found_target)
  {
    emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::WARNING, "Failed to detect the target..."));
    return;
  }

  unsigned n_dof = y0.size();
  arma::vec ones_ndof = arma::vec().ones(n_dof);
  arma::vec O_ndof = arma::vec().zeros(n_dof);

  if (log_data) unconstr_data.clear();

  double t = 0;
  double dt = robot->getCtrlCycle();
  double s = t/tau;
  double s_dot = 1/tau;
  double s_ddot = 0;
  arma::vec y = y0;
  arma::vec y_dot = O_ndof;
  arma::vec y_ddot = O_ndof;

  // gmp->setScaleMethod( gmp_::TrajScale::Ptr( new gmp_::TrajScale_Prop(n_dof) ) );
  gmp->setY0(y0);
  gmp->setGoal(yg);

  robot->update();

  // =========  Simulation loop  =========
  while (true)
  {
    if ( !sim && !isOk() ) goto end_loop;

    yg = getTargetPosition(&found_target);
    if (!found_target) PRINT_WARNING_MSG("Failed to detect the target... Keeping previous position.\n");

    gmp->setGoal(yg);

    // --------  Stopping criteria  --------
    if (s > 1.0) break;

    //if (s >= 1) { s = 1; s_dot = 0; s_ddot = 0; }

    arma::vec y = gmp->getYd(s);
    arma::vec y_dot = gmp->getYdDot(s, s_dot);
    arma::vec y_ddot = gmp->getYdDDot(s, s_dot, s_ddot);

    if (!sim)
    {
      arma::vec V_cmd = arma::vec().zeros(6);
      V_cmd.subvec(0,2) = y_dot;
      if (use_click) robot->setTaskVelocity(V_cmd, y, robot->getTaskOrientation());
      else robot->setTaskVelocity(V_cmd);
      robot->update();
    }
   
    // --------  Log data  --------
    if (log_data) unconstr_data.add(t, y, y_dot, y_ddot);

    // --------  Numerical integration  --------
    t = t + dt;
    s = s + s_dot*dt;
    s_dot = s_dot + s_ddot*dt;
  }

  if (!sim) emit gui->execFinishSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished!"));

  end_loop:
  //robot->setTaskVelocity(arma::vec().zeros(6));
  if (!sim) main_ctrl->setMode(rw_::IDLE);

  return;
}

void GmpMpcController::viewUnconstrainedPath(bool view)
{
  if (view)
  {
    runUncostr(true);
    arma::mat Pos = unconstr_data.P_data;
    rviz_pub->publishPath(Pos, rviz_::Color::MAGENTA, 0.02, "uncostr_path");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("uncostr_path");
    rviz_pub->drawnow();
  }
}

void GmpMpcController::plotUnconstrainedResults()
{
  if (unconstr_data.isEmpty())
  {
    showWarningMsg("There are no data to plot...");
    return;
  }

  arma::rowvec& Time       = unconstr_data.Time;
  arma::mat&    P_data     = unconstr_data.P_data;
  arma::mat&    dP_data    = unconstr_data.Pdot_data;
  arma::mat&    ddP_data   = unconstr_data.Pddot_data;

  // ===========  plot position  ===========
  pl_::Figure *p_fig;
  pl_::Axes *ax;

  std::vector<std::string> titles = { "x coordinate", "y coordinate", "z coordinate" };

  double tf = this->tau;
  arma::rowvec t_lim = { Time(0), tf };
  arma::rowvec t_target = {tf};
  for (int i=0;i<3;i++)
  {
    p_fig = pl_::figure("", {500, 700});
    p_fig->setAxes(3,1);

    ax = p_fig->getAxes(0);
    ax->hold(true);
    ax->plot(Time, P_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {pos_lim(i,0),pos_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {pos_lim(i,1),pos_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({target_pos(i)}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->title(titles[i], pl_::FontSize_,16);
    ax->drawnow();

    ax = p_fig->getAxes(1);
    ax->hold(true);
    ax->plot(Time, dP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {vel_lim(i,0),vel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {vel_lim(i,1),vel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->drawnow();

    ax = p_fig->getAxes(2);
    ax->hold(true);
    ax->plot(Time, ddP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {accel_lim(i,0),accel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {accel_lim(i,1),accel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->xlabel("time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }

}

// =========  Repulsive forces  =========

arma::vec rep_force(const arma::vec &y, const arma::vec &y_lim, double gain, double d0)
{
  int n_dof = y.size();

  arma::vec x = y-y_lim;
  arma::vec x_norm = arma::abs(y-y_lim);

  arma::vec f = arma::vec().zeros(n_dof);
  for (int i=0; i<n_dof; i++)
  {
    if (x_norm(i) < d0)
    {
      double e = (d0 - x_norm(i))*x(i)/x_norm(i);
      double psi = std::pow(x_norm(i) - d0, 2) / std::pow(d0,2);
      f(i) = ( 2*gain / ( std::pow(d0,2)*(1-psi) ) ) * std::log(1/(1-psi))*e;
    }
  }

  return f;
        
}

#include <boost/numeric/odeint.hpp>
// using namespace boost::numeric::odeint;
typedef std::vector< double > ode_state;
typedef boost::numeric::odeint::euler< ode_state > euler_stepper_type;
typedef boost::numeric::odeint::runge_kutta_cash_karp54< ode_state > error_stepper_type;
typedef boost::numeric::odeint::controlled_runge_kutta< error_stepper_type > controlled_stepper_type;

void rep_force_ode_fun(const ode_state &state, ode_state &state_dot, double t, 
                      gmp_::GMP *gmp, double tau, 
                      const arma::vec &y_min, const arma::vec &y_max, 
                      const arma::vec &ydot_min, const arma::vec &ydot_max, const GmpMpcController::RepForceSettings &cfg)
{  
  bool pos_on = cfg.pos_on;
  bool vel_on = cfg.vel_on;
  double kp = cfg.kp;
  double kv = cfg.kv;
  double K = cfg.K;
  double D = cfg.D;
  double d0 = cfg.d0;

  double xd_dot = 1/tau;

  // unpack state
  double x = state[0];
  double x_dot = state[1];
  arma::vec y = { state[2], state[3], state[4] };
  arma::vec z = { state[5], state[6], state[7] };
  
  arma::vec y_dot = z;
  // x_dot = x_dot;
  double x_ddot = -60*(x_dot - xd_dot);

  arma::vec yd = gmp->getYd(x);
  arma::vec yd_dot = gmp->getYdDot(x, x_dot);
  arma::vec yd_ddot = gmp->getYdDDot(x, x_dot, x_ddot);

  arma::vec fv = rep_force(y_dot,ydot_max,kv,d0) + rep_force(y_dot,ydot_min,kv,d0);
  arma::vec fp = rep_force(y,y_max,kp,d0) + rep_force(y,y_min,kp,d0);

  arma::vec z_dot = -K*(y - yd) - D*(z - yd_dot) + yd_ddot + vel_on*fv + pos_on*fp;
  y_dot = z; // + 0*fp;

  // pack state
  state_dot = {x_dot, x_ddot, y_dot(0), y_dot(1), y_dot(2), z_dot(0), z_dot(1), z_dot(2)};
}

void GmpMpcController::runRepForces(bool sim)
{
  ExecResultMsg result = loadParams(default_config_file);
  if (result.getType() != ExecResultMsg::INFO)
  {
    emit gui->execStoppedSignal(result);
    return;
  }

  robot->update();
  arma::vec y0 = robot->getTaskPosition();
  bool found_target;
  arma::vec yg = getTargetPosition(&found_target);
  if (!found_target)
  {
    emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::WARNING, "Failed to detect the target..."));
    return;
  }

  unsigned n_dof = y0.size();
  arma::vec O_ndof = arma::vec().zeros(n_dof);

  if (log_data) rep_force_data.clear();

  double t = 0;
  double dt = robot->getCtrlCycle(); //0.002;
  double s = t/tau;
  double s_dot = 1/tau;
  double s_ddot = 0;
  arma::vec y = y0;
  arma::vec y_dot = O_ndof;
  arma::vec y_ddot = O_ndof;

  double pos_cost = 0;
  double vel_cost = 0;
  int n_data = 0;

  // gmp->setScaleMethod( gmp_::TrajScale::Ptr( new gmp_::TrajScale_Prop(n_dof) ) );
  gmp->setY0(y0);
  gmp->setGoal(yg);

  arma::rowvec elaps_t_data;

  arma::wall_clock timer, timer2;
  timer.tic();

  if (!sim) main_ctrl->setMode(rw_::CART_VEL_CTRL);
  if (!sim) robot->update();

  arma::vec y_dot_prev = arma::vec().zeros(n_dof);

  arma::vec y_min = pos_lim.col(0);
  arma::vec y_max = pos_lim.col(1);

  arma::vec ydot_min = vel_lim.col(0);
  arma::vec ydot_max = vel_lim.col(1);

  arma::vec yddot_min = accel_lim.col(0);
  arma::vec yddot_max = accel_lim.col(1);

  auto ode_fun = std::bind(rep_force_ode_fun,  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
                          gmp.get(), tau, y_min, y_max, ydot_min, ydot_max, rep_force_cfg);

  controlled_stepper_type controlled_stepper;
  
  ode_state state = {s, s_dot, y(0), y(1), y(2), y_dot(0), y_dot(1), y_dot(2)};

  euler_stepper_type euler_stepper;

  std::function<void(ode_state &state, double t, double dt)> integrate_fun;
  if ( rep_force_cfg.int_method.compare("euler")==0 )
    integrate_fun = [&euler_stepper, &ode_fun](ode_state &state, double t, double dt){ boost::numeric::odeint::integrate_const(euler_stepper, ode_fun, state, t, t+dt, dt); };
  else if ( rep_force_cfg.int_method.compare("ode")==0 )
    integrate_fun = [&controlled_stepper, &ode_fun](ode_state &state, double t, double dt){ boost::numeric::odeint::integrate_adaptive(controlled_stepper, ode_fun, state, t, t+dt, dt); };
  else
  {
     emit gui->execStoppedSignal(ExecResultMsg(ExecResultMsg::ERROR, "Unrecognized integration method: '" + rep_force_cfg.int_method + "'..."));
     goto end_loop;
  }

  // =========  Simulation loop  =========
  while (true)
  {
    if ( !sim && !isOk() ) goto end_loop;

    yg = getTargetPosition(&found_target);
    if (!found_target) PRINT_WARNING_MSG("Failed to detect the target... Keeping previous position.\n");

    gmp->setGoal(yg);
    // --------  Stopping criteria  --------
    if (s > 1.0) break;

    ode_state state_dot;
    ode_fun(state, state_dot, t);
    y_ddot = { state_dot[5], state_dot[6], state_dot[7] };

    if (!sim)
    {
      arma::vec V_cmd = arma::vec().zeros(6);
      V_cmd.subvec(0,2) = y_dot;
      if (use_click) robot->setTaskVelocity(V_cmd, y, robot->getTaskOrientation());
      else robot->setTaskVelocity(V_cmd);
      robot->update();
    }
    
    pos_cost += arma::norm( y - gmp->getYd(s) );
    vel_cost += arma::norm( y_dot - gmp->getYdDot(s,s_dot) );
    n_data++;

    // --------  Log data  --------
    
    // y = robot->getTaskPosition();
    // y_dot = robot->getTaskVelocity();
    // y_ddot = (y_dot - y_dot_prev) / dt;
    // y_dot_prev = y_dot;

    if (log_data) rep_force_data.add(t, y, y_dot, y_ddot);

    timer2.tic();

    // --------  Numerical integration  --------
    t = t + dt;
    integrate_fun(state, t, dt);
    s = state[0];
    s_dot = state[1];
    y = { state[2], state[3], state[4] };
    y_dot = { state[5], state[6], state[7] };

    elaps_t_data = arma::join_horiz( elaps_t_data, arma::vec({timer2.toc()*1000}) );
  }

  pos_cost = std::sqrt( pos_cost / n_data);
  vel_cost = std::sqrt( vel_cost / n_data);

  //robot->setTaskVelocity(arma::vec().zeros(6));

  if (!sim) emit gui->execFinishSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished!"));

  end_loop:

  if (!sim) main_ctrl->setMode(rw_::IDLE);

  double elaps_t_ms = timer.toc()*1000;
  PRINT_INFO_MSG("===> Repulsive forces finished! Elaps time: " + std::to_string(elaps_t_ms) + " ms\n");

  if (!elaps_t_data.empty())
  {
    double mean_elaps_t = arma::mean(elaps_t_data);
    double std_elaps_t = arma::stddev(elaps_t_data);
    double max_elaps_t = arma::max(elaps_t_data);
    double min_elaps_t = arma::min(elaps_t_data);

    std::cerr << "======= Elapsed time (ms) ======\n";
    std::cerr << "std_range: [" << std::max(0.,mean_elaps_t-std_elaps_t) << " -  " << mean_elaps_t + std_elaps_t <<"]\n";
    std::cerr << "mean     : " << mean_elaps_t << " +/- " << std_elaps_t <<"\n";
    std::cerr << "min      : " << min_elaps_t << "\n";
    std::cerr << "max      : " << max_elaps_t << "\n";
    std::cerr << "==============================\n";

    std::cerr << "======= Final state errors (m) ======\n";
    std::cerr << "pos_err = " << arma::norm(yg - y) << "\n";
    std::cerr << "vel_err = " << arma::norm(y_dot) << "\n";
    std::cerr << "accel_err = " << arma::norm(y_ddot) << "\n";
    std::cerr << "==============================\n";

    std::cerr << "======= Cost function (m) ======\n";
    std::cerr << "RMSE_pos = " << pos_cost << "\n";
    std::cerr << "RMSE_vel = " << vel_cost << "\n";
    std::cerr << "==============================\n";
  }
  
}

void GmpMpcController::viewRepForcesPath(bool view)
{
  if (view)
  {
    std::thread thr(&GmpMpcController::runRepForces, this, true);
    makeThreadRT(thr);
    if (thr.joinable()) thr.join();

    arma::mat Pos = rep_force_data.P_data;
    rviz_pub->publishPath(Pos, rviz_::Color::PURPLE, 0.02, "rep_force_path");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("rep_force_path");
    rviz_pub->drawnow();
  }
}

void GmpMpcController::plotRepForcesResults()
{
  if (rep_force_data.isEmpty())
  {
    showWarningMsg("There are no data to plot...");
    return;
  }

  arma::rowvec& Time       = rep_force_data.Time;
  arma::mat&    P_data     = rep_force_data.P_data;
  arma::mat&    dP_data    = rep_force_data.Pdot_data;
  arma::mat&    ddP_data   = rep_force_data.Pddot_data;

  // ===========  plot position  ===========
  pl_::Figure *p_fig;
  pl_::Axes *ax;

  std::vector<std::string> titles = { "x coordinate", "y coordinate", "z coordinate" };

  double tf = this->tau;
  arma::rowvec t_lim = { Time(0), tf };
  arma::rowvec t_target = {tf};
  for (int i=0;i<3;i++)
  {
    p_fig = pl_::figure("", {500, 700});
    p_fig->setAxes(3,1);

    ax = p_fig->getAxes(0);
    ax->hold(true);
    ax->plot(Time, P_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {pos_lim(i,0),pos_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {pos_lim(i,1),pos_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({target_pos(i)}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->title(titles[i], pl_::FontSize_,16);
    ax->drawnow();

    ax = p_fig->getAxes(1);
    ax->hold(true);
    ax->plot(Time, dP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {vel_lim(i,0),vel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {vel_lim(i,1),vel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->drawnow();

    ax = p_fig->getAxes(2);
    ax->hold(true);
    ax->plot(Time, ddP_data.row(i), pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
    ax->plot(t_lim, {accel_lim(i,0),accel_lim(i,0)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot(t_lim, {accel_lim(i,1),accel_lim(i,1)}, pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::DashLine, pl_::Color_, pl_::RED);
    ax->plot({tf}, arma::rowvec({0}), pl_::LineWidth_,3.0, pl_::LineStyle_,pl_::NoLine, pl_::Color_, pl_::RED, pl_::MarkerStyle_, pl_::ssCross, pl_::MarkerSize_, 12);
    ax->xlabel("time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }

}

ExecResultMsg GmpMpcController::loadRepForceParams(const std::string &path)
{
  try
  {
    YAML::Node config = YAML::LoadFile(path);
    // load MPC settings
    YAML::Node node;
    if ( !YAML::getParam(config, "rep_force_settings", node) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings'...");
    if (!node.IsMap()) throw std::runtime_error(GmpMpcController_fun_"'rep_force_settings' must be a struct...");
    if ( !YAML::getParam(node, "d0", rep_force_cfg.d0) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.d0'...");
    if ( !YAML::getParam(node, "pos_on", rep_force_cfg.pos_on) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.pos_on'...");
    if ( !YAML::getParam(node, "vel_on", rep_force_cfg.vel_on) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.vel_on'...");
    if ( !YAML::getParam(node, "kp", rep_force_cfg.kp) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.kp'...");
    if ( !YAML::getParam(node, "kv", rep_force_cfg.kv) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.kv'...");
    if ( !YAML::getParam(node, "K", rep_force_cfg.K) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.K'...");
    if ( !YAML::getParam(node, "D", rep_force_cfg.D) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.D'...");
    if ( !YAML::getParam(node, "int_method", rep_force_cfg.int_method) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'rep_force_settings.int_method'...");

    return ExecResultMsg(ExecResultMsg::INFO, "Loaded Rep-force params successfully");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, GmpMpcController_fun_ + e.what()); }
}


// =========  Execution  =========

void GmpMpcController::startExec()
{
  std::thread thr(&GmpMpcController::run, this);
  makeThreadRT(thr);
  thr.detach();
}

bool GmpMpcController::stopExec()
{
  run_on = false;
  return stop_sem.wait_for(500);
}

// ---------------------------

bool GmpMpcController::initExperiment()
{ 
  return exp->init(); 
}

void GmpMpcController::startExperiment()
{
  std::thread thr([this]()
  { 
    run_on = true;
    exp->execute();
    run_on = false;
    stop_sem.notify();
  });
  makeThreadRT(thr);
  thr.detach();
}

bool GmpMpcController::stopExperiment()
{
  run_on = false;
  return stop_sem.wait_for(500);
}


void GmpMpcController::gotoStartPose()
{
  std::thread thr([this]()
  {
    emit gui->gotoStartJointsPosFinishedSignal( main_ctrl->moveToJointsPosition(q_start) );
  });
  makeThreadRT(thr);
  thr.detach();
}

// ===============  Rviz View ==============

void GmpMpcController::viewPosBounds(bool view)
{
  if (view)
  {
    if (pos_lim.n_rows != 3 || pos_lim.n_cols != 2)
      PRINT_WARNING_MSG(GmpMpcController_fun_ + "pos limits matrix is either unassigned or the dimensions are incorrect...\n");
    
    double x_min = pos_lim(0,0);
    double x_max = pos_lim(0,1);

    double y_min = pos_lim(1,0);
    double y_max = pos_lim(1,1);

    double z_min = pos_lim(2,0);
    double z_max = pos_lim(2,1);

    Eigen::Vector3d center( (x_min+x_max)/2 , (y_min+y_max)/2, (z_min+z_max)/2 );

    rviz_pub->publishCuboid( center, Eigen::Quaterniond(1,0,0,0), x_max-x_min, y_max-y_min, z_max-z_min, rviz_::Color(1,0,0,0.1), "pos_bounds");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("pos_bounds");
    rviz_pub->drawnow();
  }
}

void GmpMpcController::viewTargetPose(bool view)
{
  if (view)
  {
    Eigen::Vector3d pos( target_pos(0), target_pos(1), target_pos(2) );
    Eigen::Quaterniond orient( target_quat(0), target_quat(1), target_quat(2), target_quat(3) );
    rviz_pub->publishFrame(pos, orient, 1.3, "target_pose");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("target_pose");
    rviz_pub->drawnow();
  }
}

void GmpMpcController::clearAllMarkers()
{
  rviz_pub->deleteAllMarkers();
  rviz_pub->drawnow();
}

// ================== AprilTag listener ==================
bool GmpMpcController::initAprilTagListenerFromFile(const std::string &tag_listener_cfg_file, std::string *err_msg)
{
  
  // std::string tag_detections_topic;
  // std::map<std::string, std::vector<int> > op_tag_map;
  arma::mat Tf_robot_cam;
  std::string Tf_parent_link;
  std::string Tf_child_link;
  std::string camera_base_link;
  // bool publish_detections_to_tf;

  try
  {
    YAML::Node config = YAML::LoadFile(tag_listener_cfg_file);

    // // ------------  parse 'operations_tags_map'  --------------
    // YAML::Node operations_tags_node;
    // if ( !YAML::getParam(config, "operations_tags_map", operations_tags_node) )
    //   throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'operations_tags_map'...");
    // if ( !operations_tags_node.IsSequence() )
    //   throw std::runtime_error(GmpMpcController_fun_+"'operations_tags_map' must be an array...");

    // for (int i=0; i<operations_tags_node.size(); i++)
    // {
    //   std::string i_str = std::to_string(i);
    //   const YAML::Node &node_i = operations_tags_node[i];
    //   if ( !node_i.IsMap() )
    //     throw std::runtime_error(GmpMpcController_fun_+"'operations_tags_map[" + i_str + "]' must be a struct...");

    //   std::string name;
    //   int op_id;
    //   std::vector<int> tag_ids;

    //   if ( !YAML::getParam(node_i, "name", name) )
    //     throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'operations_tags_map[" + i_str + "].name'...");
    //   if ( !YAML::getParam(node_i, "tags", tag_ids) )
    //     throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'operations_tags_map[" + i_str + "].tag_ids'...");

    //   op_tag_map[name] = tag_ids;
    // }
    // --------------------------

    // ------------  parse 'camera_tf'  --------------
    YAML::Node camera_tf_node;
    if ( !YAML::getParam(config, "camera_tf", camera_tf_node) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'camera_tf'...");
    if ( !camera_tf_node.IsMap() )
        throw std::runtime_error(GmpMpcController_fun_+"'camera_tf' must be a struct...");

    if ( !YAML::getParam(camera_tf_node, "transform", Tf_robot_cam) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'camera_tf.transform'...");

    if ( !YAML::getParam(camera_tf_node, "parent_link", Tf_parent_link) )
      PRINT_WARNING_MSG(GmpMpcController_fun_+"Failed to load param 'camera_tf.parent_link'...");

    if ( !YAML::getParam(camera_tf_node, "child_link", Tf_child_link) )
      PRINT_WARNING_MSG(GmpMpcController_fun_+"Failed to load param 'camera_tf.child_link'...");
    if ( !YAML::getParam(camera_tf_node, "camera_base_link", camera_base_link) )
      PRINT_WARNING_MSG(GmpMpcController_fun_+"Failed to load param 'camera_tf.camera_base_link'...");

    // --------------------------

    // if ( !YAML::getParam(config, "tag_detections_topic", tag_detections_topic) )
    //   throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'tag_detections_topic'...");

    // if ( !YAML::getParam(config, "publish_detections_to_tf", publish_detections_to_tf) )
    //   throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'publish_detections_to_tf'...");

  }
  catch (std::exception &e)
  {
    if (err_msg) *err_msg = GmpMpcController_fun_ + "[Apriltag listener will not be created]: " + e.what();
    return false;
  }

  updateCameraPoseInRviz(Tf_robot_cam, Tf_parent_link, Tf_child_link, camera_base_link);

  tag_listener.reset( new apriltag_ros::AprilTagListener(tag_listener_cfg_file, "operations_tags_map") );
  // tag_listener.reset( new apriltag_ros::AprilTagListener(tag_detections_topic, op_tag_map) );
  tag_listener->setTagsTrasform(Tf_robot_cam, Tf_parent_link, Tf_child_link);
  // tag_listener->publishDetectionsToTf(publish_detections_to_tf);

  for (const std::string &name : tag_listener->getOperationNames())
  {
    op_names.push_back(name);
    pub_op_tags_map[name] = false;
  }

  return true;
}


ExecResultMsg GmpMpcController::loadAprilTagListener(const std::string &config_file)
{
  std::string err_msg;
  bool success = initAprilTagListenerFromFile(config_file, &err_msg);
  if (success) return ExecResultMsg(ExecResultMsg::INFO, "Loaded apriltag listener successfully!");
  else return ExecResultMsg(ExecResultMsg::ERROR, err_msg);
}

ExecResultMsg GmpMpcController::killAprilTagListener()
{
  tag_listener.reset();
  return ExecResultMsg(ExecResultMsg::INFO, " Apriltag listener was killed!");
}

ExecResultMsg GmpMpcController::loadViaPointsConfig(const std::string &path)
{
  try
  {
    YAML::Node config = YAML::LoadFile(path);

    // ------------  parse 'apriltags'  --------------
    YAML::Node tags_node;
    if ( !YAML::getParam(config, "apriltags", tags_node) )
      throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'apriltags'...");
    if ( !tags_node.IsSequence() )
      throw std::runtime_error(GmpMpcController_fun_+"'apriltags' must be an array...");

    for (int i=0; i<tags_node.size(); i++)
    {
      std::string i_str = std::to_string(i);
      const YAML::Node &node_i = tags_node[i];
      if ( !node_i.IsMap() )
        throw std::runtime_error(GmpMpcController_fun_+"'apriltags[" + i_str + "]' must be a struct...");

      ViaPoint vp;

      if ( !YAML::getParam(node_i, "id", vp.tag_id) )
        throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'apriltags[" + i_str + "].id'...");

      if ( !YAML::getParam(node_i, "tx", vp.tx) )
        throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'apriltags[" + i_str + "].tx'...");

      if ( !YAML::getParam(node_i, "err_tol", vp.err_tol) )
        throw std::runtime_error(GmpMpcController_fun_+"Failed to load param 'apriltags[" + i_str + "].err_tol'...");

      via_points_config[vp.tag_id] = vp;
    }

  }
  catch (std::exception &e)
  {
    return ExecResultMsg(ExecResultMsg::ERROR, e.what() );
  }

  return ExecResultMsg(ExecResultMsg::INFO, "Loaded apriltag via-points!");

}

void GmpMpcController::publishDetectionsToTf(bool set)
{
  if (tag_listener) tag_listener->publishDetectionsToTf(set);
  else throw std::runtime_error(GmpMpcController_fun_ + "'tag_listener' is not initialized...");
}

void GmpMpcController::publishOperationTags(const std::string &op_name, bool set)
{
  auto it = pub_op_tags_map.find(op_name);
  if (it == pub_op_tags_map.end()) return;

  bool *flag = &(it->second);

  if (*flag == set) return;

  *flag = set;

  if (*flag)
  {
    std::thread([this, op_name, flag]()
    {
      tf::TransformBroadcaster tf_br;
      while (*flag)
      {
        for (const apriltag_ros::AprilTagListener::TagDetection &tag : tag_listener->getOperationDetections(op_name) )
        {
          tf::Vector3 pos( tag.pos.x(), tag.pos.y(), tag.pos.z() );
          tf::Quaternion quat( tag.quat.x(), tag.quat.y(), tag.quat.z(), tag.quat.w());

          tf::StampedTransform tag_tf;
          tag_tf.setOrigin( pos );
          tag_tf.setRotation( quat );
          tag_tf.stamp_ = ros::Time::now();
          tag_tf.frame_id_ = this->base_link;
          tag_tf.child_frame_id_ = op_name + "_tag_" + std::to_string(tag.tag_id);
          tf_br.sendTransform(tag_tf);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }).detach();
  }
}

void GmpMpcController::updateCameraPoseInRviz(const arma::mat &Tf_base_camOpt, const std::string &base, const std::string &cam_opt, const std::string &cam_base)
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
  // tf::TransformListener tf_listener;
  // try{
  //   tf_listener.lookupTransform(cam_base, cam_opt, ros::Time(0), T_camBase_camOpt);
  // }
  // catch (tf::TransformException e){
  //   PRINT_WARNING_MSG(GmpMpcController_fun_ + e.what() + "\n");
  // }

  tf::Transform T_base_camBase = T_base_camOpt * T_camBase_camOpt.inverse();
  tf::TransformBroadcaster().sendTransform(tf::StampedTransform(T_base_camBase, ros::Time::now(), base, cam_base));
}

// =========================================================

void GmpMpcController::printElapsTime(const arma::rowvec &elaps_t_data, const std::string &title)
{
  double mean_elaps_t = arma::mean(elaps_t_data);
  double std_elaps_t = arma::stddev(elaps_t_data);
  double max_elaps_t = arma::max(elaps_t_data);
  double min_elaps_t = arma::min(elaps_t_data);

  if (!title.empty()) std::cerr << title;
  std::cerr << "std_range: [" << std::max(0.,mean_elaps_t-std_elaps_t) << " -  " << mean_elaps_t + std_elaps_t <<"]\n";
  std::cerr << "mean     : " << mean_elaps_t << " +/- " << std_elaps_t <<"\n";
  std::cerr << "min      : " << min_elaps_t << "\n";
  std::cerr << "max      : " << max_elaps_t << "\n";
  std::cerr << "==============================\n";
}

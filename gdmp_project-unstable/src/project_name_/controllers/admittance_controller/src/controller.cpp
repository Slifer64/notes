#include <admittance_controller/controller.h>

#include <iostream>
#include <io_lib/xml_parser.h>
#include <io_lib/file_io.h>
#include <plot_lib/qt_plot.h>
#include <math_lib/math_lib.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>

#include <QPushButton>

using namespace as64_;

#define AdmittanceController_fun_ std::string("[AdmittanceController::") + __func__ + "]: "


AdmittanceController::AdmittanceController(MainController *main_ctrl, const std::string &ctrl_name)
: Controller(main_ctrl, ctrl_name)
{
  // Gb_::train_ctrl = this;

  this->main_ctrl = main_ctrl;
  this->robot = main_ctrl->robot.get();

  ros::NodeHandle nh("~");

  if (!nh.getParam("admittance_controller_data_path", default_data_path)) default_data_path = "/config/";
  default_data_path = ros::package::getPath("admittance_controller") + "/" + default_data_path;

  std::string robot_descr_param;
  if (!nh.getParam("robot_description_name", robot_descr_param)) throw std::runtime_error("\33[1m\33[31mFailed to load param 'robot_description_name'...\33[0m\n");
  std::string base_link;
  if (!nh.getParam("base_link", base_link)) throw std::runtime_error("\33[1m\33[31mFailed to load param 'base_link'...\33[0m\n");
  std::string tool_link;
  if (!nh.getParam("tool_link", tool_link)) throw std::runtime_error("\33[1m\33[31mFailed to load param 'tool_link'...\33[0m\n");

  chain.initFromParam(robot_descr_param, base_link, tool_link);
  chain.setInvKinematicsSolver(robo_::NR_JL, robo_::PINV_GIVENS);
  // chain.setInvKinematicsSolver(robo_::LMA);
}

QPushButton *AdmittanceController::createGui(MainWindow *parent)
{
  QPushButton *btn = new QPushButton(this->ctrl_name.c_str());
  AdmittanceWin *gui = new AdmittanceWin(this, parent);
  QObject::connect( btn, &QPushButton::pressed, parent, [gui](){ gui->launch(); });
  return btn;
}

AdmittanceController::~AdmittanceController()
{}

// ===============================================================

void AdmittanceController::start()
{
  ExecResultMsg msg = loadParams();
  if (msg.getType() != ExecResultMsg::INFO)
  {
    emit gui->stopCtrlSignal(msg);
    return;
  }
  this->setMode(ctrl_mode);
  run_ = true;
  execute();
  exec_stop_sem.notify();
}

ExecResultMsg AdmittanceController::stop()
{
  run_ = false;
  if ( exec_stop_sem.wait_for(1500) ) return ExecResultMsg(ExecResultMsg::INFO, "Controller stopped!");
  else return ExecResultMsg(ExecResultMsg::WARNING, "Time-out reached on waiting for controller to stop...");
}

void AdmittanceController::execute()
{
  arma::vec M = {Mp, Mp, Mp, Mo, Mo, Mo};
  arma::vec D = {Dp_max, Dp_max, Dp_max, Do_max, Do_max, Do_max};

  double Ts = robot->getCtrlCycle();
  arma::vec q = robot->getJointsPosition();
  arma::vec P = robot->getTaskPosition();
  arma::vec Q = robot->getTaskOrientation();
  arma::vec V = arma::vec().zeros(6);
  arma::vec V_dot = arma::vec().zeros(6);
  arma::vec Fext = arma::vec().zeros(6);

  double dq_norm = 0;

  int count = 0;
  
  while (run_)
  {
    if (!robot->isOk())
    {
      std::cerr << "\33[1\33[31m" << "The robot is not ok...\n" << "\33[0m" << std::flush;
      emit gui->stopCtrlSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting execution..."));
      break;
    }
    robot->update();

    q = robot->getJointsPosition();
    arma::vec F_msr = robot->getTaskWrench();
    Fext = a_f*F_msr + (1 - a_f)*Fext;

    applyEnableDoFs(Fext);

    arma::vec p_dot = V.subvec(0, 2);
    arma::vec omega = V.subvec(3, 5);
    arma::vec force = Fext.subvec(0,2);
    arma::vec torque = Fext.subvec(3,5);
 
    double vp = std::max( arma::dot(p_dot, force) + arma::dot(omega, torque), 0.0 );
    double Dp = Dp_min + (Dp_max - Dp_min)*std::exp(-lambda_P*vp);

    double vo = vp;
    double Do = Do_min + (Do_max - Do_min)*std::exp(-lambda_P*vo);

    D = {Dp, Dp, Dp, Do, Do, Do};


    V_dot = (-D%V + Fext) / M;

    P = P + V.subvec(0,2)*Ts;
    Q = math_::quatProd(math_::quatExp(V.subvec(3,5)*Ts), Q);
    V = V + V_dot*Ts;

    applyThreshold(V, cart_vel_lim);

    if (ctrl_mode == rw_::JOINT_POS_CONTROL)
    {
      bool found_sol;
      arma::mat pose = arma::mat().eye(4,4);
      pose.submat(0,0,2,2) = math_::quat2rotm(Q);
      pose.submat(0,3,2,3) = P;
      arma::vec q_cmd = chain.getJointPositions(pose, q, &found_sol);
      if (!found_sol)
      {
        emit gui->stopCtrlSignal(ExecResultMsg(ExecResultMsg::ERROR, "Failed to find inverse kinematic solution...\n Aborting execution..."));
        break;
      }
      robot->setJointsPosition(q_cmd);
    }
    else if (ctrl_mode == rw_::JOINT_VEL_CONTROL)
    {
      arma::mat J = chain.getJacobian(q);
      arma::vec q_dot = arma::pinv(J, J_sigma_min)*V;
      // applyThreshold(q_dot, joint_vel_lim);

      double a_dq_min = 0.0005;
      double a_dq_max = 1;
      double sigma;
      {
        arma::mat U, V;
        arma::vec s;
        arma::svd_econ(U, s, V, J);
        sigma = s.back(); // singular values are in descending order
      }
      double sigma_t = 0.2;
      double a_dq = a_dq_max;
      if (sigma < sigma_t) a_dq = a_dq_min + (a_dq_max - a_dq_min)*std::exp(-0.1/(sigma_t - sigma));

      // a_dq = a_dq_min + (a_dq_max - a_dq_min)*std::exp(-0.1/sigma);

      dq_norm = a_dq*arma::norm(q_dot) + (1-a_dq)*dq_norm;
      q_dot = dq_norm * q_dot / (arma::norm(q_dot) + 1e-16);

      double sat = false;
      if (arma::norm(q_dot)/q_dot.size() > joint_vel_lim[0])
      {
        sat = true;
        q_dot = joint_vel_lim[0] * q_dot.size() * q_dot / arma::norm(q_dot);
      }

      if (count++ % 500 == 0 || sat)
      {
        if (sat) std::cerr << "*** SAT ***\n";
        std::cerr << " sigma_min = " << sigma << "\n";
        std::cerr << " a_dq = " << a_dq << "\n";
        std::cerr << " norm(q_dot) = " << arma::norm(q_dot) << "\n";
        std::cerr << "====================\n";
      }

      robot->setJointsVelocity(q_dot);
    }
    else if (ctrl_mode == rw_::CART_VEL_CTRL)
    {
      robot->setTaskVelocity(V); //, P, Q); // use of CLICK is not necessary
    }

  }

  robot->setTaskVelocity(arma::vec().zeros(6));

  run_ = false;
}

ExecResultMsg AdmittanceController::loadParams()
{
  try{
    YAML::Node nh = YAML::LoadFile(default_data_path + "ctrl_params.yaml");

    if ( !YAML::getParam(nh, "Mp", Mp) ) throw std::runtime_error("Failed to load param \"Mp\"...");
    if ( !YAML::getParam(nh, "Dp_min", Dp_min) ) throw std::runtime_error("Failed to load param \"Dp_min\"...");
    if ( !YAML::getParam(nh, "Dp_max", Dp_max) ) throw std::runtime_error("Failed to load param \"Dp_max\"...");
    if ( !YAML::getParam(nh, "Mo", Mo) ) throw std::runtime_error("Failed to load param \"Mo\"...");
    if ( !YAML::getParam(nh, "Do_min", Do_min) ) throw std::runtime_error("Failed to load param \"Do_min\"...");
    if ( !YAML::getParam(nh, "Do_max", Do_max) ) throw std::runtime_error("Failed to load param \"Do_max\"...");
    if ( !YAML::getParam(nh, "lambda_P", lambda_P) ) throw std::runtime_error("Failed to load param \"lambda_P\"...");
    if ( !YAML::getParam(nh, "a_f", a_f) ) throw std::runtime_error("Failed to load param \"a_f\"...");
    if ( !YAML::getParam(nh, "J_sigma_min", J_sigma_min) ) throw std::runtime_error("Failed to load param \"J_sigma_min\"...");

    if ( !YAML::getParam(nh, "joint_vel_lim", joint_vel_lim) ) throw std::runtime_error("Failed to load param \"joint_vel_lim\"...");
    if ( !YAML::getParam(nh, "cart_vel_lim", cart_vel_lim) ) throw std::runtime_error("Failed to load param \"cart_vel_lim\"...");

    YAML::Node enabled_dofs_node;
    if ( !YAML::getParam(nh, "enabled_dofs", enabled_dofs_node) ) throw std::runtime_error("Failed to load param \"enabled_dofs\"...");
    if ( !YAML::getParam(enabled_dofs_node, "dofs", enabled_dofs) ) throw std::runtime_error("Failed to load param \"enabled_dofs.dofs\"...");
    std::string frame;
    if ( !YAML::getParam(enabled_dofs_node, "frame", frame) ) throw std::runtime_error("Failed to load param \"enabled_dofs.frame\"...");
    if (frame.compare("base") == 0) applyEnableDoFs = [this](arma::vec &Fext){ Fext = Fext % enabled_dofs; };
    else if (frame.compare("tool") == 0)
    {
      applyEnableDoFs = [this](arma::vec &Fext)
      {
        arma::mat R = chain.getRotm(robot->getJointsPosition());
        Fext.subvec(0,2) = R.t() * Fext.subvec(0,2);
        Fext.subvec(3,5) = R.t() * Fext.subvec(3,5);
        Fext = Fext % enabled_dofs;
        Fext.subvec(0,2) = R * Fext.subvec(0,2);
        Fext.subvec(3,5) = R * Fext.subvec(3,5);
      };
    }
    else throw std::runtime_error("Unrecognized frame '" + frame + "' for enabled dofs");

    return ExecResultMsg(ExecResultMsg::INFO, "Loaded params successfully!");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, std::string("Failed to load params:") + e.what()); }
  
}

void AdmittanceController::plotTrainData()
{
//   if (!isTrainData())
//   {
//     showWarningMsg("The training data are empty!");
//     return;
//   }

//   arma::rowvec Time = demo_data.getTime();
//   arma::mat P_data = demo_data.getPos();
//   arma::mat dP_data = demo_data.getVel();
//   arma::mat Q_data = demo_data.getQuat();
//   arma::mat vRot_data = demo_data.getRotVel();


//   int n_data = Time.size();
//   arma::rowvec vel_norm(n_data);
//   for (int j=0; j<n_data; j++) vel_norm(j) = arma::norm(dP_data.col(j));

//   pl_::Figure *fig_ = pl_::figure("", {500, 600});
//   fig_->setAxes(1,1);
//   pl_::Axes *ax = fig_->getAxes(0);
//   ax->plot(Time, vel_norm, pl_::LineWidth_,2.0, pl_::Color_,pl_::RED);
//   ax->title("Translational Velocity L2 norm", pl_::FontSize_,16, pl_::FontWeight_,pl_::DemiBold);
//   ax->xlabel("Time [s]", pl_::FontSize_,14);
//   // ax->drawnow();
//   // fig_->drawnow();

//   // ===========  plot position  ===========
//   pl_::Figure *p_fig = pl_::figure("", {500, 600});


//   p_fig->setAxes(2,1);

//   ax = p_fig->getAxes(0);
//   ax->hold(true);
//   // ax->xlabel("time [s]");
//   for (int i=0;i<3;i++)
//   {
//     pl_::Graph *graph = ax->plot(Time, P_data.row(i), pl_::LineWidth_,2.0);
// //    graph->setLineWidth(1);
// //    graph->setLineStyle(DashDotLine);
// //    graph->setMarkerStyle(ssStar);
// //    graph->setMarkerSize(10);
//   }
//   ax->title("Cartesian Position", pl_::FontSize_,16, pl_::FontWeight_,pl_::DemiBold);
//   ax->ylabel("Position [m]", pl_::FontSize_,14);
//   ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
//   // ax->drawnow();

//   ax = p_fig->getAxes(1);
//   ax->hold(true);
//   // ax->xlabel("time [s]");
//   for (int i=0;i<3;i++) ax->plot(Time, dP_data.row(i), pl_::LineWidth_,2.0);
//   // ax->title("Cartesian Velocity");
//   ax->ylabel("Velocity [m/s]", pl_::FontSize_,14);
//   ax->xlabel("Time [s]", pl_::FontSize_,14);
//   ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
//   // ax->drawnow();

//   // p_fig->drawnow();

//   // ===========  plot orientation  ===========
//   n_data = Q_data.n_cols;
//   arma::mat q_data(3, n_data);
//   arma::vec Q0 = Q_data.col(0);
//   for (int j=0; j<n_data; j++) q_data.col(j) = model->quatToOrient(Q_data.col(j), Q0);

//   pl_::Figure *q_fig = pl_::figure("", {500, 600});
//   q_fig->setAxes(2,1);

//   ax = q_fig->getAxes(0);
//   ax->hold(true);
//   // ax->xlabel("time [s]");
//   for (int i=0;i<3;i++) ax->plot(Time, q_data.row(i), pl_::LineWidth_,2.0);
//   ax->title("Cartesian Orientation", pl_::FontSize_,16, pl_::FontWeight_,pl_::DemiBold);
//   ax->ylabel("Quaternion error", pl_::FontSize_,14);
//   ax->legend({"eq_x", "eq_y", "eq_z"}, pl_::FontSize_,15);
//   // ax->drawnow();

//   ax = q_fig->getAxes(1);
//   ax->hold(true);
//   // ax->xlabel("time [s]");
//   for (int i=0;i<3;i++) ax->plot(Time, vRot_data.row(i), pl_::LineWidth_,2.0);
//   ax->ylabel("Angular Vel [rad/s]", pl_::FontSize_,14);
//   ax->xlabel("Time [s]", pl_::FontSize_,14);
//   ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
//   // ax->drawnow();

//   // q_fig->drawnow();

//   pl_::drawnow();
}


ExecResultMsg AdmittanceController::setCtrlMethod(const std::string &ctrl_mode_str)
{  
  if (ctrl_mode_str.compare("JOINT_POS") == 0) ctrl_mode = rw_::JOINT_POS_CONTROL;
  else if (ctrl_mode_str.compare("JOINT_VEL") == 0) ctrl_mode = rw_::JOINT_VEL_CONTROL;
  else if (ctrl_mode_str.compare("CART_VEL") == 0) ctrl_mode = rw_::CART_VEL_CTRL;
  else return ExecResultMsg(ExecResultMsg::ERROR, "Unsupported ctrl mode '" + ctrl_mode_str + "'...\n Aborting execution...");

  return ExecResultMsg(ExecResultMsg::INFO, "Ctrl mode changed successfully!");
}

void AdmittanceController::applyThreshold(arma::vec &v, const arma::vec &lim)
{
  if (arma::norm(v)/v.size() > lim[0])
  {
    std::cerr << arma::norm(v)/v.size() << " :=> " << lim[0] << "\n";
    v = lim[0] * v.size() * v / arma::norm(v);
  }
  // for (int i=0; i<v.size(); i++) v[i] = std::max(std::min(v[i], lim[i]), -lim[i]);
}
#ifndef $_PROJECT_384$_ADMITTANCE_CONTROLLER_H
#define $_PROJECT_384$_ADMITTANCE_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <admittance_controller/gui.h>

#include <robot_wrapper/robot.h>
#include <main_controller/main_controller.h>
#include <main_controller/controller.h>

#include <robo_lib/kinematic_chain.h>

using namespace as64_;

class AdmittanceController : public Controller
{
public:

  AdmittanceController(MainController *main_ctrl, const std::string &ctrl_name);
  ~AdmittanceController();

  QPushButton *createGui(MainWindow *parent) override;

  std::string getDefaultPath() const { return default_data_path; }

public:

  ros::NodeHandle node;

  AdmittanceWin *gui;

  void start();
  ExecResultMsg stop();

  void execute();

  void plotTrainData();

  ExecResultMsg loadParams();

  ExecResultMsg setCtrlMethod(const std::string &ctrl_mode);

  rw_::Mode ctrl_mode;

  bool run_;

  // void setAdmittanceEnabledDoFs(const std::vector<bool> &enabled_dofs) { robot->adm_ctrl->setEnabledDoFs(enabled_dofs); }

  std::string default_data_path;

  thr_::Semaphore exec_stop_sem;

  double Mp; // cart pos stiffness
  double Dp_min;
  double Dp_max;

  double Mo; // cart orient stiffness
  double Do_min; // cart orient damping
  double Do_max;

  double lambda_P; // variable damping change rate

  double a_f; // filtering coeff for wrench (1: no filtering)

  double J_sigma_min;

  arma::uvec enabled_dofs;
  std::function<void(arma::vec &)> applyEnableDoFs;

  arma::vec joint_vel_lim;
  arma::vec cart_vel_lim;

  void applyThreshold(arma::vec &v, const arma::vec &lim);

  robo_::KinematicChain chain;

};

#endif // $_PROJECT_384$_ADMITTANCE_CONTROLLER_H

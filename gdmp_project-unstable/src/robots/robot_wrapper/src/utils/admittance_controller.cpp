#include <robot_wrapper/utils/admittance_controller.h>

#include <robot_wrapper/robot.h>
#include <robot_wrapper/utils/xml_parser.h>

#include <armadillo>

#include <ros/package.h>

namespace rw_
{

#define AdmCtrl_fun_ std::string("[AdmittanceCtrl::") + __func__ + "]: "

AdmittanceCtrl::AdmittanceCtrl(rw_::Robot *robot)
{
  robot_ = robot;

  enabled_dofs = {0, 0, 0, 0, 0, 0};
}

void AdmittanceCtrl::init()
{
  p = robot_->getTaskPosition();
  p_dot = p_ddot = arma::vec().zeros(3);

  Q = robot_->getTaskOrientation();
  omega = omega_dot = arma::vec().zeros(3);

  Fext_prev = arma::vec().zeros(6);

  std::string params_file = ros::package::getPath("robot_wrapper") + "/config/adm_params.yaml";
  loadParams(params_file);
}

void AdmittanceCtrl::stop()
{
  p_ddot = p_dot = omega_dot = omega = arma::vec().zeros(3);
  ret_vel = arma::vec().zeros(6);
}

void AdmittanceCtrl::loadParams(const std::string &filename)
{
  rw_::XmlParser parser(filename);

  if (!parser.getParam("Mp",Mp)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Mp\"...");
  if (!parser.getParam("Dp_min",Dp_min)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Dp_min\"...");
  if (!parser.getParam("Dp_max",Dp_max)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Dp_max\"...");
  if (!parser.getParam("Mo",Mo)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Mo\"...");
  if (!parser.getParam("Do_min",Do_min)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Do_min\"...");
  if (!parser.getParam("Do_max",Do_max)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Do_max\"...");
  if (!parser.getParam("lambda_P",lambda_P)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"lambda_P\"...");
  if (!parser.getParam("a_f",a_f)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"a_f\"...");

  // If I uncomment the next line, the 'enabled_dofs' has update issues... Veeery strange...
  // if (!parser.getParam("enabled_dofs",enabled_dofs)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"enabled_dofs\"...");
}

void AdmittanceCtrl::update()
{
  static int count = 0;

  arma::vec active_dofs;
  {
    std::unique_lock<std::mutex> lck(enabled_dofs_mtx);
    active_dofs = enabled_dofs;
    // if (count++ % 1000 == 0)
    // {
    //   std::cerr << "enabled_dofs = " << enabled_dofs.t() << "\n";
    //   std::cerr << "active_dofs = " << active_dofs.t() << "\n";
    // }
    
  }

  arma::vec Fext = robot_->getCompTaskWrench();
  Fext = Fext % active_dofs;
  Fext = (1-a_f)*Fext_prev + a_f*Fext;
  Fext_prev = Fext;

  //std::cerr << "||F_ext|| = " << arma::norm(Fext) << "\n";

  arma::vec force = Fext.subvec(0,2);
  arma::vec torque = Fext.subvec(3,5);

  double vp = std::max( arma::dot(p_dot, force) + arma::dot(omega, torque), 0.0 );
  double Dp = Dp_min + (Dp_max - Dp_min)*std::exp(-lambda_P*vp);

  double vo = vp;
  double Do = Do_min + (Do_max - Do_min)*std::exp(-lambda_P*vo);

  p_ddot = (-Dp*p_dot + force) / Mp;
  omega_dot = (-Do*omega + torque) / Mo;

  this->ret_vel = arma::join_vert(p_dot, omega);

  double dt = robot_->getCtrlCycle();
  p = p + p_dot*dt;
  p_dot = p_dot + p_ddot*dt;
  Q = quatProd(quatExp(omega*dt), Q);
  omega = omega + omega_dot*dt;

  // we could also use KLICK, but in admittance control its not really necessary
}


arma::vec AdmittanceCtrl::getVelocity() const
{
  return ret_vel;
}

void AdmittanceCtrl::setEnabledDoFs(const std::vector<bool> &active_dofs) 
{ 
  std::unique_lock<std::mutex> lck(enabled_dofs_mtx);

  for (int i=0; i<enabled_dofs.size(); i++) enabled_dofs[i] = active_dofs[i];

  // std::cerr << "==> AdmittanceCtrl after: on_dofs = " << enabled_dofs.t() << "\n";
  // std::cerr << "----------------------------\n";
}


} // namespace rw_

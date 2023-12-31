#ifndef AS64_ROBOT_WRAPPER_ADMITTANCE_CONTROLLER_H
#define AS64_ROBOT_WRAPPER_ADMITTANCE_CONTROLLER_H

#include <robot_wrapper/robot.h>

#include <string>
#include <armadillo>
#include <mutex>

namespace rw_
{

class Robot; // forward declaration

class AdmittanceCtrl
{
public:
  AdmittanceCtrl(rw_::Robot *robot);

  void init();

  void update();

  void loadParams(const std::string &filename);

  arma::vec getVelocity() const;

  void stop();

  void setEnabledDoFs(const std::vector<bool> &enabled_dofs);

private:

  rw_::Robot *robot_;

  // controller state
  arma::vec p, p_dot, p_ddot; // cart pos state
  arma::vec Q, omega, omega_dot; // cart orient state

  // controller params
  double Mp; // cart pos stiffness
  double Dp_min; // cart pos damping
  double Dp_max;

  double Mo; // cart orient stiffness
  double Do_min; // cart orient damping
  double Do_max;

  double lambda_P;

  double a_f; // filtering coeff for wrench
  arma::vec Fext_prev;

  arma::vec ret_vel; // the admittance cart velocity calculated after each update

  std::mutex enabled_dofs_mtx;
  arma::vec enabled_dofs;

};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_ADMITTANCE_CONTROLLER_H

#ifndef $_PROJECT_384$_TARGET_MODEL_H
#define $_PROJECT_384$_TARGET_MODEL_H

#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <gmp_model/model.h>

using namespace as64_;

class TargetModel : public Model
{

public:
  TargetModel();
  ~TargetModel();

  virtual bool load(const std::string &path, std::string *err_msg=0);

  virtual void train(const std::string &train_method, const arma::rowvec &Time, const arma::mat &Pos, const arma::mat &Quat);

  virtual void setTargetPose(const arma::vec &Pg, const arma::vec &Qg);
  virtual void setInitialPose(const arma::vec &P0, const arma::vec &Q0);

  virtual arma::vec getRefPos(double x) const;
  virtual arma::vec getRefVel(double x, double x_dot) const;
  virtual arma::vec getRefAccel(double x, double x_dot, double x_ddot) const;
  virtual arma::vec getRefOrient(double x) const;
  virtual arma::vec getRefQuat(double x) const;
  virtual arma::vec getRefRotVel(double x, double x_dot) const;

  virtual arma::vec calcAccel(const arma::vec &pos, const arma::vec &vel, double x, double x_dot, double x_ddot) const;
  virtual arma::vec calcOrientAccel(const arma::vec &q, const arma::vec &q_dot, double x, double x_dot, double x_ddot) const;

  virtual arma::vec getMotionDir(double x) const;

  virtual void updatePosition(double x, const arma::vec &P);
  virtual void updateqLogQuat(double x, const arma::vec &q);

private:

  // transform position from world to target
  arma::vec w2t_pos(const arma::vec &pos) const;
  arma::vec w2t_vel(const arma::vec &vel) const;

  arma::vec t2w_pos(const arma::vec &pos) const;
  arma::vec t2w_vel(const arma::vec &vel) const;

  // transform orientation from world to target
  arma::vec w2t_quat(const arma::vec &quat) const;
  arma::vec t2w_quat(const arma::vec &quat) const;

  arma::vec Qt; // Qt = inv(Qg), transform orientation using: Qt*Q
  arma::mat Rt; // Rt = Rg.t(), rotate position to target using: Rt*(p - pg);
  arma::vec p_o; // p_o = Pg, origin

  arma::vec P0_exec;
  arma::vec Q0_exec;

};

#endif // $_PROJECT_384$_TARGET_MODEL_H

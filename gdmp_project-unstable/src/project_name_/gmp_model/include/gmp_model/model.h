#ifndef $_PROJECT_384$_MODEL_H
#define $_PROJECT_384$_MODEL_H

#include <string>
#include <vector>
#include <memory>
#include <armadillo>
#include <Eigen/Dense>

#include <gmp_lib/gmp_lib.h>
#include <thread_lib/mtx_var.h>

using namespace as64_;

class Model
{
public:

  enum Type
  {
    STD=0,
    TARGET=1,
  };

  Model();
  ~Model();

  std::string getDefaultPath() const { return default_model_path; }

  bool isTrained() const { return is_trained_(); };
  virtual bool load(const std::string &path, std::string *err_msg=0);
  virtual bool save(const std::string &path, std::string *err_msg=0);

  void initialize(unsigned N_kernels);

  virtual void train(const std::string &train_method, const arma::rowvec &Time, const arma::mat &Pos, const arma::mat &Quat);

  virtual void setTargetPose(const arma::vec &Pg, const arma::vec &Qg);
  virtual void setInitialPose(const arma::vec &P0, const arma::vec &Q0);

  void setPosStiffness(double Kp);
  void setPosDamping(double Dp);
  void setOrientStiffness(double Ko);
  void setOrientDamping(double Do);

  virtual arma::vec getRefPos(double x) const;
  virtual arma::vec getRefVel(double x, double x_dot) const;
  virtual arma::vec getRefAccel(double x, double x_dot, double x_ddot) const;
  virtual arma::vec getRefOrient(double x) const;
  virtual arma::vec getRefOrientDot(double x, double x_dot) const;
  virtual arma::vec getRefOrientDDot(double x, double x_dot, double x_ddot) const;
  virtual arma::vec getRefQuat(double x) const;
  virtual arma::vec getRefRotVel(double x, double x_dot) const;

  arma::vec getTrainFinalPos() const { return Pf; }
  arma::vec getTrainFinalQuat() const { return Qf / arma::norm(Qf); }
  arma::vec getTrainInitPos() const { return P0; }
  arma::vec getTrainInitQuat() const { return Q0 / arma::norm(Q0); }

  double getTrainDuration() const { return Tf; }

  arma::rowvec getXtrainData() const { return x_train_data; }

  virtual arma::vec calcAccel(const arma::vec &pos, const arma::vec &vel, double x, double x_dot, double x_ddot) const;
  virtual arma::vec calcOrientAccel(const arma::vec &q, const arma::vec &q_dot, double x, double x_dot, double x_ddot) const;

  virtual arma::vec getMotionDir(double x) const;

  static arma::vec quatToOrient(const arma::vec &Q, const arma::vec &Q0);
  static arma::vec orientToQuat(const arma::vec &q, const arma::vec &Q0);
  static arma::vec orientVelToRotVel(const arma::vec &q_dot, const arma::vec &Q, const arma::vec &Q0);

  arma::mat getPositionPath(const arma::vec &P0, const arma::vec &Pg, const arma::vec &Q0, const arma::vec &Qg, double duration);
  arma::mat getQuatPath(const arma::vec &P0, const arma::vec &Pg, const arma::vec &Q0, const arma::vec &Qg, double duration);

  Model::Type getType() const { return type; }

  void initUpdate();
  virtual void updatePosition(double x, const arma::vec &P);
  virtual void updateqLogQuat(double x, const arma::vec &q);

  void updatePositionParams();
  void updateOrientationParams();

protected:

  gmp_::GMP gmp_p;
  gmp_::GMP gmp_p_cp;
  gmp_::GMP_Update::Ptr gmp_p_up;
  
  gmp_::GMPo gmp_o;
  gmp_::GMPo gmp_o_cp;
  gmp_::GMPo_Update::Ptr gmp_o_up;

  arma::rowvec x_train_data; // keep training timestamps to use them for inititializing Sw in update
  
  bool is_initialized_;

  // Initial and final values of training (desired) trajectory
  arma::vec P0;
  arma::vec Pf;
  arma::vec Q0;
  arma::vec Qf;

  double Tf;

  // commanded new initial and target pose
  arma::vec Q_init;
  arma::vec Q_target;

  thr_::MtxVar<bool> is_trained_;

  std::string default_model_path;

  Type type;
};

#endif // $_PROJECT_384$_MODEL_H

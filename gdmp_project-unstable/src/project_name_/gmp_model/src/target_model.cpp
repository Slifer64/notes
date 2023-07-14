#include <gmp_model/target_model.h>
#include <io_lib/print_utils.h>
#include <io_lib/file_io.h>
#include <math_lib/math_lib.h>

#include <ros/package.h>

#define TargetModel_fun_ std::string("[TargetModel::") + __func__ + "]: "

TargetModel::TargetModel()
{
  type = Model::Type::TARGET;

  Qt = {1,0,0,0};
  Rt = math_::quat2rotm(Qt);
  p_o = {0,0,0};
}

TargetModel::~TargetModel()
{

}

void TargetModel::train(const std::string &train_method, const arma::rowvec &Time, const arma::mat &Pos, const arma::mat &Quat)
{
  int n_data = Time.size();
  int i_end = n_data-1;

  setInitialPose(Pos.col(0), Quat.col(0));
  setTargetPose(Pos.col(i_end), Quat.col(i_end));

  // change coordinates to the target
  arma::mat Pos_t(3,n_data);
  arma::mat Quat_t(4,n_data);
  for (int j=0; j<n_data; j++)
  {
    Pos_t.col(j) = w2t_pos(Pos.col(j));
    Quat_t.col(j) = w2t_quat(Quat.col(j));
  }

  Model::train(train_method, Time, Pos_t, Quat_t);

  this->Pf = t2w_pos(this->Pf);
  this->Qf = t2w_quat(this->Qf);
  this->P0 = t2w_pos(this->P0);
  this->Q0 = t2w_quat(this->Q0);
}

void TargetModel::setTargetPose(const arma::vec &Pg, const arma::vec &Qg)
{
  Qt = math_::quatInv(Qg);
  Rt = math_::quat2rotm(Qt);
  p_o = Pg;

  Model::setTargetPose(w2t_pos(Pg), w2t_quat(Qg));

  setInitialPose(P0_exec, Q0_exec); // reset these since the target has changed
}

void TargetModel::setInitialPose(const arma::vec &P0, const arma::vec &Q0)
{
  P0_exec = P0;
  Q0_exec = Q0;
  Model::setInitialPose(w2t_pos(P0), w2t_quat(Q0));
}

arma::vec TargetModel::getRefPos(double x) const
{
  return t2w_pos(Model::getRefPos(x));
}

arma::vec TargetModel::getRefVel(double x, double x_dot) const
{
  return t2w_vel(Model::getRefVel(x, x_dot));
}

arma::vec TargetModel::getRefAccel(double x, double x_dot, double x_ddot) const
{
  return t2w_vel(Model::getRefAccel(x, x_dot, x_ddot));
}

arma::vec TargetModel::getRefOrient(double x) const
{
  return Model::getRefOrient(x);
  // return quatToOrient( getRefQuat(x) , t2w_quat(gmp_o->getQ0()) );
}

arma::vec TargetModel::getRefQuat(double x) const
{
  return Model::getRefQuat(x);
  // return t2w_quat(Model::getRefQuat(x));
}

arma::vec TargetModel::getRefRotVel(double x, double x_dot) const
{
  return Model::getRefRotVel(x, x_dot);
  // return t2w_vel(Model::getRefRotVel(x, x_dot));
}

arma::vec TargetModel::getMotionDir(double x) const
{
  return t2w_vel(Model::getMotionDir(x));
}

arma::vec TargetModel::calcAccel(const arma::vec &pos, const arma::vec &vel, double x, double x_dot, double x_ddot) const
{
  // return Model::calcAccel(pos, vel, x, x_dot, x_ddot);
  return t2w_vel(Model::calcAccel(w2t_pos(pos), w2t_vel(vel), x, x_dot, x_ddot));
}

arma::vec TargetModel::calcOrientAccel(const arma::vec &q, const arma::vec &q_dot, double x, double x_dot, double x_ddot) const
{
  return Model::calcOrientAccel(q, q_dot, x, x_dot, x_ddot);

//   arma::vec Q0_t = gmp_o->getQ0();
//
//   arma::vec Q0_w = t2w_quat(Q0_t);
//   arma::vec Q_w = orientToQuat(q, Q0_w);
//   arma::vec Q1_w = gmp_::GMPo::quatTf(Q_w, Q0_w);
//   arma::vec vRot_w = gmp_::GMPo::qdot2rotVel(q_dot, Q1_w);
//
//   arma::vec Q_t = w2t_quat(Q_w);
//   arma::vec vRot_t = w2t_vel(vRot_w);
//
// //  arma::vec Q1_t = gmp_::GMPo::quatTf(Q_t, Q0_t);
// //  arma::vec q_t = quatToOrient(Q_t, Q0_t);
// //  arma::vec qdot_t = gmp_::GMPo::rotVel2qdot(vRot_t, Q1_t);
// //  arma::vec qddot_t = Model::calcOrientAccel(q_t, qdot_t, x, x_dot, x_ddot);
// //  arma::vec rotAccel_t = gmp_::GMPo::qddot2rotAccel(qddot_t, vRot_t, Q1_t);
//
//   arma::vec Qg = {1, 0, 0, 0};
//   return t2w_vel( gmp_o->calcRotAccel(gmp_::Phase(x,x_dot,x_ddot), Q_t, vRot_t, Qg) );
}

void TargetModel::updatePosition(double x, const arma::vec &P)
{
  Model::updatePosition(x, w2t_pos(P));
}

void TargetModel::updateqLogQuat(double x, const arma::vec &q)
{
  Model::updateqLogQuat(x, q);
}

arma::vec TargetModel::w2t_pos(const arma::vec &pos) const
{
  // return pos;
  return Rt*(pos - p_o);
}

arma::vec TargetModel::w2t_vel(const arma::vec &vel) const
{
  // return vel;
  return Rt*vel;
}

arma::vec TargetModel::t2w_pos(const arma::vec &pos) const
{
  // return pos;
  return Rt.t()*pos + p_o;
}

arma::vec TargetModel::t2w_vel(const arma::vec &vel) const
{
  // return vel;
  return Rt.t()*vel;
}

arma::vec TargetModel::w2t_quat(const arma::vec &quat) const
{
  return quat;
  // return math_::quatProd(Qt, quat);
}

arma::vec TargetModel::t2w_quat(const arma::vec &quat) const
{
  return quat;
  // return math_::quatProd(math_::quatInv(Qt), quat);
}


bool TargetModel::load(const std::string &path, std::string *err_msg)
{
  if (!Model::load(path, err_msg)) return false;

  P0_exec = getTrainInitPos();
  Q0_exec = getTrainFinalQuat();
  setTargetPose(getTrainFinalPos(), getTrainFinalQuat());
  return true;
}

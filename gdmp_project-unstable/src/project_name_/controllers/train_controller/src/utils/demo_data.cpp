#include <train_controller/utils/demo_data.h>
#include <math_lib/math_lib.h>


using namespace as64_;

#define DemoData_fun_ std::string("[DemoData::") + __func__ + "]: "

DemoData::DemoData()
{

}

arma::vec DemoData::getPos(const std::string &sind) const
{
  if (sind == "end") return P_data.col(i_end());
  else throw std::runtime_error("[DemoData::getPos]: Unrecognized input \"" + sind + "\n");
}

arma::vec DemoData::getVel(const std::string &sind) const
{
  if (sind == "end") return getVel(i_end());
  else throw std::runtime_error("[DemoData::getVel]: Unrecognized input \"" + sind + "\n");
}

arma::vec DemoData::getQuat(const std::string &sind) const
{
  if (sind == "end") return Q_data.col(i_end());
  else throw std::runtime_error("[DemoData::getQuat]: Unrecognized input \"" + sind + "\n");
}

arma::vec DemoData::getRotVel(const std::string &sind) const
{
  if (sind == "end") return getRotVel(i_end());
  else throw std::runtime_error("[DemoData::getRotVel]: Unrecognized input \"" + sind + "\n");
}

arma::vec DemoData::getJointPos(const std::string &sind) const
{
  if (sind == "end") return joint_pos_data.col(i_end());
  else throw std::runtime_error("[DemoData::getJointPos]: Unrecognized input \"" + sind + "\n");
}

void DemoData::add(double t, const arma::vec &pos, const arma::vec &quat, const arma::vec &joint_pos)
{
  Time = arma::join_horiz(Time,arma::vec{t});
  P_data = arma::join_horiz(P_data,pos);
  Q_data = arma::join_horiz(Q_data,quat);
  joint_pos_data = arma::join_horiz(joint_pos_data,joint_pos);
}

void DemoData::clear()
{
  this_cp.reset();

  Time.clear();
  P_data.clear();
  Q_data.clear();
  joint_pos_data.clear();
}

void DemoData::removeStops(double vel_thres)
{
  if (getDataSize() == 0)
  {
    std::cerr << "\033[1m" << "\033[33m" << "[WARNING]: " << (DemoData_fun_ + "The data are empty...\n") << "\033[0m";
    return;
  }

  int n_data = Time.size();
  double Ts = Time(1) - Time(0);
  arma::vec vel_norm(n_data);
  for (int j=0; j<n_data-1; j++) vel_norm(j) = arma::norm( ( P_data.col(j+1) - P_data.col(j) ) / Ts );
  vel_norm(n_data-1) = 0.5*vel_norm(n_data-2);

  arma::uvec ind = arma::find(vel_norm > vel_thres);

  Time = arma::linspace<arma::rowvec>(0, ind.size()-1, ind.size())*Ts;
  // Time = Time(ind).t();
  // Time += -Time(0);

  P_data = P_data.cols(ind);
  Q_data = Q_data.cols(ind);
}

void DemoData::trim(double pos_thres, double orient_thres)
{
  if (getDataSize() == 0)
  {
    std::cerr << "\033[1m" << "\033[33m" << "[WARNING]: " << (DemoData_fun_ + "The data are empty...\n") << "\033[0m";
    return;
  }

  if (!this_cp) this_cp.reset( new DemoData(*this) );

  int n_data = Time.size();
  int j1 = 0;
  int j2 = n_data-1;

  for (int j=0; j<n_data-1; j++)
  {
    double dt = Time(j+1) - Time(j);
    if ( arma::norm( (P_data.col(j+1) - P_data.col(0)) /*/ dt*/ ) >= pos_thres  ||  
        arma::norm( math_::quatLog(math_::quatDiff(Q_data.col(j+1), Q_data.col(0))) /*/ dt*/ ) >= orient_thres )
    { j1 = j; break; }
  }

  for (int j=n_data-2; j>j1; j--)
  {
    double dt = Time(j+1) - Time(j);
    if ( arma::norm( (P_data.col(n_data-1) - P_data.col(j)) /*/ dt*/ ) >= pos_thres  ||  
        arma::norm( math_::quatLog(math_::quatDiff(Q_data.col(n_data-1), Q_data.col(j))) /*/ dt*/ ) >= orient_thres )
    { j2 = j+1; break; }
  }

  std::cerr << "====== Trim demo =======\n";
  std::cerr << "Initial: [" << 0 << ", " << n_data-1 << "]\n";
  std::cerr << "Final: [" << j1 << ", " << j2 << "]\n";



  Time = Time.subvec(j1, j2) - Time(j1);
  P_data = P_data.cols(j1,j2);
  Q_data = Q_data.cols(j1,j2);
  joint_pos_data = joint_pos_data.cols(j1,j2);
}

void DemoData::undoTrim()
{
  // if (this_cp) *this = *this_cp.get();
  if (this_cp)
  {
    this->Time = this_cp->Time;
    this->P_data = this_cp->P_data;
    this->Q_data = this_cp->Q_data;
    this->joint_pos_data = this_cp->joint_pos_data;
  }
  else std::cerr << "\033[1m" << "\033[33m" << "[WARNING]: " << (DemoData_fun_ + "cannot undo trim...\n") << "\033[0m";

}

arma::vec DemoData::getVel(int i) const
{
  if (i > i_end()-1) return arma::vec().zeros(3);
  return (getPos(i+1)-getPos(i)) / ( getTime(i+1) - getTime(i) );
}

arma::vec DemoData::getRotVel(int i) const
{
  if (i > i_end()-1) return arma::vec().zeros(3);
  return math_::quatLog( math_::quatDiff(getQuat(i+1), getQuat(i)) ) / ( getTime(i+1) - getTime(i) );
}

void DemoData::setData(const arma::rowvec &Timed, const arma::mat &Pd_data, const arma::mat &Qd_data, const arma::mat &joint_pos_data)
{
  int n_data = Timed.size();
  if (Pd_data.n_cols != n_data) throw std::runtime_error(DemoData_fun_ + "Pd_data.n_cols != Timed.size()");
  if (Pd_data.n_rows != 3) throw std::runtime_error(DemoData_fun_ + "Pd_data.n_cols != 3");

  if (Qd_data.n_cols != n_data) throw std::runtime_error(DemoData_fun_ + "Qd_data.n_cols != Timed.size()");
  if (Qd_data.n_rows != 4) throw std::runtime_error(DemoData_fun_ + "Qd_data.n_cols != 4");

  if (joint_pos_data.n_cols != n_data) throw std::runtime_error(DemoData_fun_ + "joint_pos_data.n_cols != Timed.size()");

  this->Time = Timed;
  this->P_data = Pd_data;
  this->Q_data = Qd_data;
  this->joint_pos_data = joint_pos_data;
}

arma::mat DemoData::getVel() const
{
  arma::mat dP_data(3, getDataSize());
  for (int j=0; j<dP_data.n_cols; j++) dP_data.col(j) = getVel(j);
  return dP_data;
}

arma::mat DemoData::getRotVel() const
{
  arma::mat vRot_data(3, getDataSize());
  for (int j=0; j<vRot_data.n_cols; j++) vRot_data.col(j) = getRotVel(j);
  return vRot_data;
}

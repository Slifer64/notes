#ifndef $_PROJECT_384$_DEMO_DATA_H
#define $_PROJECT_384$_DEMO_DATA_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>


class DemoData
{
public:

  DemoData();

  bool empty() const { return Time.empty(); }

  int getDataSize() const { return Time.size();}

  double getTime(int i) const { return Time(i); }
  arma::rowvec getTime() const { return Time; }
  double getDuration() const { return Time(i_end()); }

  arma::vec getPos(int i) const { return P_data.col(i); }
  arma::vec getPos(const std::string &sind) const;
  arma::mat getPos() const { return P_data; }

  arma::vec getVel(int i) const;
  arma::vec getVel(const std::string &sind) const;
  arma::mat getVel() const;

  arma::vec getQuat(int i) const { return Q_data.col(i); }
  arma::vec getQuat(const std::string &sind) const;
  arma::mat getQuat() const { return Q_data; }

  arma::vec getRotVel(int i) const;
  arma::vec getRotVel(const std::string &sind) const;
  arma::mat getRotVel() const;

  arma::vec getJointPos(int i) const { return joint_pos_data.col(i); }
  arma::vec getJointPos(const std::string &sind) const;
  arma::mat getJointPos() const { return joint_pos_data; }

  void add(double t, const arma::vec &pos, const arma::vec &quat, const arma::vec &joint_pos);

  void setData(const arma::rowvec &Timed, const arma::mat &Pd_data, const arma::mat &Qd_data, const arma::mat &joint_pos_data);

  void clear();

  void trim(double pos_thres, double orient_thres);

  void undoTrim();

  void removeStops(double vel_thres = 5e-4);

  arma::vec getInitialJointPos() const { return joint_pos_data.col(0); }
  arma::vec getFinalJointPos() const { return joint_pos_data.col(i_end()); }

private:

  int i_end() const { return getDataSize()-1; };

  std::shared_ptr<DemoData> this_cp;

  arma::rowvec Time;
  arma::mat P_data;
  arma::mat Q_data;

  arma::mat joint_pos_data;
};

#endif // $_PROJECT_384$_DEMO_DATA_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <armadillo>

#include <gmp_lib/GMP/GMP_regressor.h>
#include <gmp_lib/GMP/GMP_MPC.h>
#include <rviz_lib/rviz_marker_publisher.h>


using namespace as64_;

struct Obstacle
{
  Obstacle(const arma::vec &center, const arma::vec &axes_len, const arma::vec &tag_pos, const arma::vec &tag_quat, const std::string &name) 
  {
    this->name = name;

    arma::mat R = math_::quat2rotm(tag_quat);
    this->Sigma = R * arma::diagmat(arma::pow(axes_len,2)) * R.t();
    this->c = tag_pos + R*center;
  }

  Obstacle(const arma::vec &center, const arma::mat &Sigma, const std::string &name)
  {
    this->c = center;
    this->Sigma = Sigma;
    this->name = name;
  }

  arma::vec c;
  arma::mat Sigma;
  std::string name;
};


class GmpMpcRvizPlot
{
public:
  GmpMpcRvizPlot(const std::string &marker_pub_ns, const std::string &base_frame)
  {
    rviz_pub.reset(new rviz_::RvizMarkerPublisher(marker_pub_ns, base_frame));
  }

  GmpMpcRvizPlot()
  {
    stop();
  }

  void stop()
  {
    run_ = false;
    if (pub_thread.joinable()) pub_thread.join();
  }

  void init(unsigned N_kernels, double kernel_std_scaling, const arma::vec &y0, unsigned pub_cycle=50)
  {
    regressor.reset(new gmp_::GMP_regressor(N_kernels, kernel_std_scaling));

    y_opt_points.init(y0);
    yd_points.init(y0);

    y_init = Eigen::Vector3d(y0(0), y0(1), y0(2));

    this->pub_cycle = pub_cycle;

    std::cerr << "[GmpMpcRvizPlot::init]: Initializing...\n";

    std::vector<std::string> markes_ns = {"y_opt", "yd", "y_opt_pred", "yd_pred_points", "y_pred_points", "obst_constr", "y_target", "y_init"};
    if (published_markers)
    { 
      std::cerr << "Removing previous markers...\n";
      for (auto &ns : markes_ns) rviz_pub->deleteMarkers(ns);
      for (auto &ns : visible_obst) rviz_pub->deleteMarkers(ns);
      rviz_pub->drawnow();
      published_markers = false;
    }

    // // ----- Unconstrained trajectory -------
    // ax->plot(arma::rowvec(Pd_data.row(0)), arma::rowvec(Pd_data.row(1)), pl_::LineWidth_,3, pl_::Color_,pl_::BLUE);
    // ax->plot(arma::rowvec({Pd_data(0, 0)}), arma::rowvec({Pd_data(1, 0)}), pl_::LineWidth_,4, pl_::MarkerStyle_,pl_::ssCircle, pl_::Color_,pl_::GREEN, pl_::MarkerSize_,16);
    // ax->plot(arma::rowvec({Pd_data.row(0).back()}), arma::rowvec({Pd_data.row(1).back()}), pl_::LineWidth_,4, pl_::MarkerStyle_,pl_::ssCross, pl_::Color_,pl_::RED, pl_::MarkerSize_,16);
    // // ----- Online generated trajectory -----
    // this->p_h = ax->plot(arma::rowvec({std::numeric_limits<double>::quiet_NaN()}), arma::rowvec({std::numeric_limits<double>::quiet_NaN()}),
    //                      pl_::LineWidth_,2, pl_::Color_,pl_::MAGENTA);

    run_ = true;
    this->updated = false;

    pub_thread = std::thread([this]()
    {
      constr_exist = false;
      while (run_)
      {
        publishFun();
        std::this_thread::sleep_for(std::chrono::milliseconds(this->pub_cycle));
      }
    });
  }

  void operator()(const gmp_::GMP_MPC::Log &log, const std::vector<Obstacle> &obstacles)
  {
    std::unique_lock<std::mutex> lck(log_mtx);
    this->updated = true;
    this->log_ = log;
    this->obstacles_ = obstacles;
  }

  void publishFun()
  {
    std::unique_lock<std::mutex> lck(log_mtx);
    if (!this->updated) return;
    gmp_::GMP_MPC::Log log = this->log_;
    std::vector<Obstacle> obstacles = this->obstacles_;
    this->updated = false;
    lck.unlock();

    y_opt_points.addPoint(log.y_current);
    yd_points.addPoint(log.yd_current);

    rviz_pub->publishCylinder(y_opt_points.prev_pos, y_opt_points.current_pos, line_width/2, y_opt_color, "y_opt");
    rviz_pub->publishCylinder(yd_points.prev_pos, yd_points.current_pos, line_width/2, yd_color, "yd");

    int n_dof = log.y_current.size();

    arma::rowvec s_next = arma::linspace<arma::rowvec>(log.si_data[0], log.si_data.back(), 30);
    arma::mat P_opt_next(n_dof, s_next.size());
    for (int j=0; j<s_next.size(); j++) P_opt_next.col(j) = log.W_opt * regressor->regressVec(s_next(j));
    rviz_pub->deleteMarkers("y_opt_pred");
    rviz_pub->publishPath(P_opt_next, y_opt_pred_color, line_width, "y_opt_pred");
    
    if (constr_exist)
    {
      rviz_pub->deleteMarkers("obst_constr");
      constr_exist = false;
    }
    for (int i=0; i<log.n_e_data.size(); i+=3)
    {
      constr_exist = true;
      const arma::vec &n_e = log.n_e_data[i];
      const arma::vec &p = log.p_data[i];
      const arma::vec &p_e = log.p_e_data[i];
      rviz_pub->publishPlane(Eigen::Vector3d(p_e(0), p_e(1), p_e(2)), Eigen::Vector3d(n_e(0), n_e(1), n_e(2)), 0.07, 0.07, rviz_::Color(0,1,0,0.4), "obst_constr");
      double h = 0.06;
      arma::vec c = p_e + 0.5*h*n_e;
      rviz_pub->publishCylinder(Eigen::Vector3d(c(0), c(1), c(2)+h/2), Eigen::Vector3d(n_e(0), n_e(1), n_e(2)), h, 0.02, rviz_::Color(0,1,0,0.4), "obst_constr");
    }

    rviz_pub->deleteMarkers("yd_pred_points");
    for (auto &y : log.yd_points)
      rviz_pub->publishSphere(Eigen::Vector3d(y(0), y(1), y(2)), points_radius, rviz_::Color::CYAN, "yd_pred_points");

    rviz_pub->deleteMarkers("y_pred_points");
    for (auto &y : log.y_pred_points)
      rviz_pub->publishSphere(Eigen::Vector3d(y(0), y(1), y(2)), points_radius, rviz_::Color::MAGENTA, "y_pred_points");

    if (!visible_obst.empty()) {for (auto &ns : visible_obst) rviz_pub->deleteMarkers(ns); }
    visible_obst.clear();
    for (int i=0; i<obstacles.size(); i++)
    {
      auto &obst = obstacles[i];
      Eigen::Vector3d center(obst.c(0), obst.c(1), obst.c(2));
      Eigen::Matrix3d Sigma = Eigen::Map<Eigen::Matrix3d>(obst.Sigma.memptr());
      rviz_pub->publishEllipsoid(center, Sigma, obst_color[i%obst_color.size()], obst.name);
      visible_obst.push_back(obst.name);
    }

    Eigen::Vector3d target(log.y_target(0), log.y_target(1), log.y_target(2));
    rviz_pub->deleteMarkers("y_target");
    rviz_pub->publishSphere(target, 2*points_radius, rviz_::Color::RED, "y_target");

    rviz_pub->deleteMarkers("y_init");
    rviz_pub->publishSphere(y_init, 2*points_radius, rviz_::Color::GREEN, "y_init");

    rviz_pub->drawnow();
    published_markers = true;
  }

private:

  bool run_ = false;
  std::thread pub_thread;

  std::mutex log_mtx;
  gmp_::GMP_MPC::Log log_;
  bool updated;

  std::vector<rviz_::Color> obst_color = {rviz_::Color(0.597, 0.296, 0, 0.5), rviz_::Color(1.0, 0.5, 0, 0.5), rviz_::Color(1.0, 1.0, 0, 0.5)};

  std::vector<Obstacle> obstacles_;

  struct OnlinePathPoints
  {
    Eigen::Vector3d current_pos;
    Eigen::Vector3d prev_pos;

    void init(const arma::vec &p0)
    {
      current_pos = prev_pos = Eigen::Vector3d(p0(0), p0(1), p0(2));
    }

    void addPoint(const arma::vec &p)
    {
      prev_pos = current_pos;
      current_pos = Eigen::Vector3d(p(0), p(1), p(2));
    }
  };

  bool constr_exist = false;
  volatile bool published_markers = false;
  std::vector<std::string> visible_obst;

  double line_width = 0.02;
  double points_radius = 0.01;

  rviz_::Color y_opt_color = rviz_::Color::PURPLE;
  rviz_::Color y_opt_pred_color = rviz_::Color::PINK;
  rviz_::Color yd_color = rviz_::Color::BLUE;
  rviz_::Color yd_pred_color = rviz_::Color::CYAN;
  
  OnlinePathPoints y_opt_points;
  OnlinePathPoints yd_points;
  Eigen::Vector3d y_init;

  std::shared_ptr<rviz_::RvizMarkerPublisher> rviz_pub;

  std::shared_ptr<gmp_::GMP_regressor> regressor;

  unsigned pub_cycle;
};

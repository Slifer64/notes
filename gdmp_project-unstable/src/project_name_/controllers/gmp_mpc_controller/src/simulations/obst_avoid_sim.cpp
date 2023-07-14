#include <iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <thread>
#include <armadillo>
#include <ros/package.h>

#include <gmp_lib/io/file_io.h>
#include <gmp_lib/GMP/GMP_MPC.h>
#include <gmp_lib/CanonicalSystem/CanonicalSystem.h>
#include <gmp_lib/io/gmp_io.h>
#include <plot_lib/qt_plot.h>

using namespace as64_;

void PRINT_INFO(const std::string &msg)
{ std::cout << "\33[1m\33[32m" << msg << "\33[0m\n" << std::flush; }

void PRINT_WARNING(const std::string &msg)
{ std::cout << "\33[1m\33[33m" << msg << "\33[0m\n" << std::flush; }

void PRINT_ERROR(const std::string &msg)
{ std::cout << "\33[1m\33[31m" << msg << "\33[0m\n" << std::flush; }

void program_pause()
{
  std::cout << "\33[1m\33[34m" << "Program paused. Press [enter] to coninue..." << "\33[0m\n" << std::flush;
  std::string dummy;
  std::getline(std::cin, dummy, '\n');
}

struct Ellipsoid
{
  Ellipsoid(const arma::vec &c_, const arma::mat &Sigma_): c(c_), Sigma(Sigma_) {}
  arma::vec c;
  arma::mat Sigma;
};

struct PlotData
{
  arma::rowvec Time;
  arma::mat Pos;
  arma::mat Vel;
  arma::mat Accel;
  pl_::LineStyle linestyle;
  pl_::Color color;
  std::string legend;
  int linewidth;
};

struct Trajectory
{
  Trajectory(const arma::rowvec &Time_, const arma::mat &Pos_, const arma::mat &Vel_, const arma::mat &Accel_):
      Time(Time_), Pos(Pos_), Vel(Vel_), Accel(Accel_) {}

  arma::rowvec Time;
  arma::mat Pos;
  arma::mat Vel;
  arma::mat Accel;
};

arma::mat drawElipsoid2D(const arma::mat &Sigma, const arma::vec &c);

arma::mat getEllipseSigma(double angle, double lambda_x, double lambda_y);

Trajectory getGMPTrajectory(gmp_::GMP &gmp, const arma::vec &y0, const arma::vec &yg, double Tf, double dt);

Trajectory gmpMpcOpt(gmp_::GMP &gmp0,
                     double dt, double Tf,
                     const arma::vec &y0, const arma::vec &yg,
                     const arma::mat &pos_lim, const arma::mat &vel_lim, const arma::mat &accel_lim,
                     const std::vector<Ellipsoid> &obstacles,
                     double opt_pos, double opt_vel);


class OnlinePlot
{
public:
  OnlinePlot(const gmp_::GMP_MPC &gmp_mpc_)
  {
    this->gmp_mpc = &gmp_mpc_;
  }

  void init(const arma::mat &Pd_data,
            const arma::vec &pos_lb, const arma::vec &pos_ub,
            const std::vector<arma::mat> &obst_curves, int plot_every = 5, unsigned sleep_ms=1)
  {
    this->plot_every = plot_every;
    plt_show_count = 0;
    this->sleep_ms = sleep_ms;

    fig = pl_::figure("", {500, 700});;
    fig->setAxes(1,1);
    ax = fig->getAxes(0);

    std::vector<pl_::Color> obst_colors = {pl_::MUSTARD, pl_::PURPLE, pl_::LIGHT_BROWN, pl_::PINK, pl_::CYAN};

    // ----- pos bounds -----
    ax->plot(arma::rowvec({pos_lb(0), pos_ub(0)}), arma::rowvec({pos_lb(1), pos_lb(1)}), pl_::LineWidth_,2, pl_::Color_,pl_::LIGHT_RED);
    ax->plot(arma::rowvec({pos_lb(0), pos_ub(0)}), arma::rowvec({pos_ub(1), pos_ub(1)}), pl_::LineWidth_,2, pl_::Color_,pl_::LIGHT_RED);
    ax->plot(arma::rowvec({pos_lb(0), pos_lb(0)}), arma::rowvec({pos_lb(1), pos_ub(1)}), pl_::LineWidth_,2, pl_::Color_,pl_::LIGHT_RED);
    ax->plot(arma::rowvec({pos_ub(0), pos_ub(0)}), arma::rowvec({pos_lb(1), pos_ub(1)}), pl_::LineWidth_,2, pl_::Color_,pl_::LIGHT_RED);
    // ----- Unconstrained trajectory --------
    ax->plot(arma::rowvec(Pd_data.row(0)), arma::rowvec(Pd_data.row(1)), pl_::LineWidth_,3, pl_::Color_,pl_::BLUE);
    ax->plot(arma::rowvec({Pd_data(0, 0)}), arma::rowvec({Pd_data(1, 0)}), pl_::LineWidth_,4, pl_::MarkerStyle_,pl_::ssCircle, pl_::Color_,pl_::GREEN, pl_::MarkerSize_,16);
    ax->plot(arma::rowvec({Pd_data.row(0).back()}), arma::rowvec({Pd_data.row(1).back()}), pl_::LineWidth_,4, pl_::MarkerStyle_,pl_::ssCross, pl_::Color_,pl_::RED, pl_::MarkerSize_,16);
    // ----- Online generated trajectory -----
    this->p_h = ax->plot(arma::rowvec({std::numeric_limits<double>::quiet_NaN()}), arma::rowvec({std::numeric_limits<double>::quiet_NaN()}),
                         pl_::LineWidth_,2, pl_::Color_,pl_::MAGENTA);
    // ----- Obstacles ------
    for (int i=0; i<obst_curves.size(); i++)
      ax->plot(arma::rowvec(obst_curves[i].row(0)), arma::rowvec(obst_curves[i].row(1)), pl_::LineWidth_,2, pl_::Color_,obst_colors[i % obst_colors.size()]);
//    axis(this->ax, 'tight');
//    axis(this->ax, 'equal');
    ax->xlabel("X [m]", pl_::FontSize_,15);
    ax->ylabel("Y [m]", pl_::FontSize_,15);

    x_lim = ax->xlim();
    y_lim = ax->ylim();
  }

  void operator()(const gmp_::GMP_MPC::Log &log)
  {
    p_h->addData(log.y_current(0), log.y_current(1));

    plt_show_count++;
    if (plt_show_count < plot_every) return;

    for (auto &h : plt_handles) h->clear();
    plt_handles.clear();

    int n_dof = log.y_current.size();

//    s_next = si_data;
//    P_next = zeros(this.n_dof, length(s_next));
//    for j=1:length(s_next), P_next(:,j) = this.getYd(s_next(j)); end
//    h = plot(P_next(1,:), P_next(2,:), 'LineWidth',2, 'Color',[1 0.7 1], 'LineStyle','None', 'Marker','*', 'MarkerSize',16, 'HandleVisibility','off');
//    this.plt_handles = [this.plt_handles h];
    arma::rowvec s_next = arma::linspace<arma::rowvec>(log.si_data[0], log.si_data.back(), 30);
    arma::mat P_next(n_dof, s_next.size());
    for (int j=0; j<s_next.size(); j++) P_next.col(j) = gmp_mpc->getYd(s_next(j));
    pl_::Graph *h = ax->plot(arma::rowvec(P_next.row(0)), arma::rowvec(P_next.row(1)), pl_::LineWidth_,2, pl_::Color_,pl_::PINK);
    plt_handles.push_back(h);

    for (int i=0; i<log.n_e_data.size(); i++)
    {
      const arma::vec &n_e = log.n_e_data[i];
      const arma::vec &p1 = log.p_data[i];
      const arma::vec &p2 = log.p_e_data[i];

      arma::vec v = {n_e(1), -n_e(0)}; // tangent line direction
      arma::vec p3 = p2 - 0.1*v;
      arma::vec p4 = p2 + 0.1*v;

      arma::vec p2_n = p2 + 0.1*n_e;

      pl_::Graph *h1 = ax->plot(arma::rowvec({p2(0), p2_n(0)}), arma::rowvec({p2(1), p2_n(1)}), pl_::Color_,pl_::GREEN, pl_::LineWidth_,3);
      pl_::Graph *h2 = ax->plot(arma::rowvec({p3(0), p4(0)}), arma::rowvec({p3(1), p4(1)}), pl_::LineWidth_,2.2, pl_::Color_,pl_::CYAN);
      pl_::Graph *h3 = ax->plot(arma::rowvec({p1(0)}), arma::rowvec({p1(1)}), pl_::LineWidth_,3, pl_::MarkerStyle_,pl_::ssCross, pl_::MarkerSize_,16, pl_::Color_,pl_::RED);

      plt_handles.push_back(h1);
      plt_handles.push_back(h2);
      plt_handles.push_back(h3);
    }

    for (auto &y : log.yd_points)
    {
      pl_::Graph *h = ax->plot(arma::rowvec({y(0)}), arma::rowvec({y(1)}),
                               pl_::LineWidth_,2, pl_::MarkerStyle_,pl_::ssCross, pl_::MarkerSize_,16, pl_::Color_,pl_::CYAN);
      plt_handles.push_back(h);
    }

    for (auto &y : log.y_pred_points)
    {
      pl_::Graph *h = ax->plot(arma::rowvec({y(0)}), arma::rowvec({y(1)}),
                               pl_::LineWidth_,2, pl_::MarkerStyle_,pl_::ssCircle, pl_::MarkerSize_,16, pl_::Color_,pl_::MAGENTA);
      plt_handles.push_back(h);
    }

    ax->xlim(x_lim[0], x_lim[1]);
    ax->ylim(y_lim[0], y_lim[1]);

    ax->drawnow();
    plt_show_count = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    // program_pause();
  }

private:

  const gmp_::GMP_MPC *gmp_mpc;
  pl_::Figure *fig;
  pl_::Axes *ax;
  pl_::Graph *p_h;

  std::vector<pl_::Graph *> plt_handles;

  int plot_every;
  int plt_show_count;
  unsigned sleep_ms;

  std::array<double, 2> x_lim;
  std::array<double, 2> y_lim;

};


int main(int argc, char **argv)
{
  pl_::QtPlot::init();

  arma::rowvec sd_data;
  arma::mat Pd_data;
  PRINT_INFO("Loading train data...");
  try{
    std::string data_path = ros::package::getPath("gmp_mpc_controller") + "/data/simulation/obst_avoid_train_data.bin";
    gmp_::FileIO fid(data_path, gmp_::FileIO::in);
    fid.read("sd_data", sd_data);
    fid.read("Pd_data", Pd_data);
  }
  catch (std::exception &e)
  {
    PRINT_ERROR(e.what());
    exit(0);
  }

  unsigned n_dof = Pd_data.n_rows;

  std::vector<Ellipsoid> ellipsoid;
  ellipsoid.push_back(Ellipsoid({-0.14, 0.28}, getEllipseSigma(45, 0.19, 0.06)));
  ellipsoid.push_back(Ellipsoid({0.16, 0.61}, getEllipseSigma(-40, 0.13, 0.07)));
  std::vector<pl_::Color> ellipsoid_colors = {pl_::MUSTARD, pl_::PURPLE};

  std::vector<arma::mat> E_p;
  for (auto &e : ellipsoid) E_p.push_back(drawElipsoid2D(e.Sigma, e.c));

  PRINT_INFO("Trainig GMP model...");
  gmp_::GMP gmp(2, 30, 1.5);
  arma::vec train_err;
  gmp.train("LS", sd_data, Pd_data, &train_err);
  std::cerr << "train_err: " << train_err.t() << "\n";

  // -------- Limits ---------
  arma::mat pos_lim = { {-0.3, 0.3},
                        {-0.03, 0.77} };
  arma::mat vel_lim = arma::repmat(arma::rowvec({-0.6, 0.6}), n_dof, 1);
  arma::mat accel_lim = arma::repmat(arma::rowvec({-2 , 2}), n_dof, 1);

  // --------- Optimization objective ----------
  double opt_pos = 1.0;
  double opt_vel = 0.0;

  // -------- Initial/Final states --------
  arma::vec y0 = Pd_data.col(0);
  arma::vec yg = Pd_data.col(Pd_data.n_cols-1);
  double Tf = 5.0;
  double tau = Tf;
  double dt = 0.005;

  gmp.setY0(y0);
  gmp.setGoal(yg);

  // =============  Run  ==================
  std::vector<PlotData> data;

  // --------- Simple Execution -----------
  PRINT_INFO("DMP simulation...");
  {
    Trajectory traj = getGMPTrajectory(gmp, y0, yg, Tf, dt);
    PlotData dat;
    dat.Time = traj.Time; dat.Pos = traj.Pos; dat.Vel = traj.Vel; dat.Accel = traj.Accel;
    dat.linestyle = pl_::SolidLine; dat.color = pl_::BLUE; dat.legend = "DMP"; dat.linewidth = 4;
    data.push_back(dat);
  }

  // ---------- GMP-MPC optimization ------------
  PRINT_INFO("DMP-MPC simulation...");
  {
    Trajectory traj = gmpMpcOpt(gmp, dt, Tf, y0, yg, pos_lim, vel_lim, accel_lim, ellipsoid, opt_pos, opt_vel);
    PlotData dat;
    dat.Time = traj.Time; dat.Pos = traj.Pos; dat.Vel = traj.Vel; dat.Accel = traj.Accel;
    dat.linestyle = pl_::SolidLine; dat.color = pl_::MAGENTA; dat.legend = "DMP*"; dat.linewidth = 2;
    data.push_back(dat);
  }

  // --------- Plot results ------------
  pl_::Figure *fig;
  pl_::Axes *ax;

  fig = pl_::figure("", {500, 700});
  fig->setAxes(1,1);
  ax = fig->getAxes(0);
  ax->plot(arma::rowvec(Pd_data.row(0)), arma::rowvec(Pd_data.row(1)),
           pl_::LineWidth_,2.0, pl_::LineStyle_,pl_::SolidLine, pl_::Color_, pl_::BLUE);
  ax->plot(arma::rowvec({Pd_data(0, 0)}), arma::rowvec({Pd_data(1, 0)}),
           pl_::LineWidth_,4, pl_::MarkerStyle_, pl_::ssCircle, pl_::Color_, pl_::GREEN, pl_::MarkerSize_,16);
  ax->plot(arma::rowvec({Pd_data.row(0).back()}), arma::rowvec({Pd_data.row(1).back()}),
           pl_::LineWidth_,4, pl_::MarkerStyle_,pl_::ssCross, pl_::Color_, pl_::RED, pl_::MarkerSize_,16);
//  for (auto &dat : data)
//  {
//    ax->plot(arma::rowvec(dat.Pos.row(0)), arma::rowvec(dat.Pos.row(1)),
//             pl_::LineWidth_,dat.linewidth, pl_::LineStyle_,dat.linestyle, pl_::Color_, dat.color);
//  }
  for (int i=0; i<ellipsoid.size(); i++)
  {
    ax->plot(arma::rowvec(E_p[i].row(0)), arma::rowvec(E_p[i].row(1)),
             pl_::LineWidth_,2, pl_::Color_,ellipsoid_colors[i]);
    ax->plot(arma::rowvec(ellipsoid[i].c(0)), arma::rowvec(ellipsoid[i].c(1)),
             pl_::LineWidth_,4, pl_::MarkerStyle_, pl_::ssCross, pl_::Color_,ellipsoid_colors[i], pl_::MarkerSize_,16);
  }

  auto &dat = data[1];
  auto line_h = ax->plot(arma::rowvec(dat.Pos.row(0)(0)), arma::rowvec(dat.Pos.row(1)(0)),
                         pl_::LineWidth_,dat.linewidth, pl_::LineStyle_,dat.linestyle, pl_::Color_, dat.color);

  pl_::drawnow();

  for (int i=1; i<dat.Pos.n_cols; i++)
  {
    line_h->addData(dat.Pos.row(0)(i), dat.Pos.row(1)(i));
    ax->drawnow();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  program_pause();

}


// ===================================
// ===================================

arma::mat drawElipsoid2D(const arma::mat &Sigma, const arma::vec &c)
{
  arma::rowvec theta = arma::linspace<arma::rowvec>(0, 2*arma::datum::pi, 200);
  arma::mat L = arma::chol(Sigma, "lower");
  arma::mat p = arma::repmat(c, 1, theta.size()) + L*arma::join_vert(arma::cos(theta), arma::sin(theta));
  return p;
}


arma::mat getEllipseSigma(double angle, double lambda_x, double lambda_y)
{
  double theta = angle * arma::datum ::pi / 180;
  arma::mat R = { {std::cos(theta), -std::sin(theta)}, {std::sin(theta), std::cos(theta)} };
  arma::mat Lambda_2 = arma::diagmat( arma::vec({lambda_x, lambda_y}) );
  arma::mat Sigma = R * arma::pow(Lambda_2, 2) * R.t();
  return Sigma;
}


Trajectory getGMPTrajectory(gmp_::GMP &gmp, const arma::vec &y0, const arma::vec &yg, double Tf, double dt)
{
  unsigned n_dof = y0.size();

  std::shared_ptr<gmp_::TrajScale> traj_scale(new gmp_::TrajScale_Prop(n_dof));
  gmp.setScaleMethod(traj_scale);

  gmp.setY0(y0);
  gmp.setGoal(yg);

  arma::rowvec s_data = arma::linspace<arma::rowvec>(0, 1, std::round(Tf/dt));
  double s_dot = 1/Tf;
  double s_ddot = 0;

  int n_data = s_data.size();
  int n_dofs = y0.size();

  arma::rowvec Time = s_data * Tf;
  arma::mat P_data(n_dofs, n_data);
  arma::mat dP_data(n_dofs, n_data);
  arma::mat ddP_data(n_dofs, n_data);
  for (int j=0; j<n_data; j++)
  {
    double s = s_data(j);
    P_data.col(j) = gmp.getYd(s);
    dP_data.col(j) = gmp.getYdDot(s, s_dot);
    ddP_data.col(j) = gmp.getYdDDot(s, s_dot, s_ddot);
  }

  return Trajectory(Time, P_data, dP_data, ddP_data);
}


Trajectory gmpMpcOpt(gmp_::GMP &gmp,
                     double dt, double Tf,
                     const arma::vec &y0, const arma::vec &yg,
                     const arma::mat &pos_lim, const arma::mat &vel_lim, const arma::mat &accel_lim,
                     const std::vector<Ellipsoid> &obstacles,
                     double opt_pos, double opt_vel)
{
  unsigned n_dof = y0.size();
  
  arma::vec O_ndof = arma::vec().zeros(n_dof);
  
  double t = 0;
  gmp_::CanonicalSystem can_sys(Tf, 30);
  arma::vec y = y0;
  arma::vec y_dot = O_ndof;
  arma::vec y_ddot = O_ndof;
  
  int N_horizon = 10;
  double pred_time_step = 0.1;
  int N_kernels = 30;
  double kernels_std_scaling = 1.5;
  
  arma::vec final_state_err_tol = 1e-1* arma::vec({1e-3, 1e-2, 1e-1});

  std::array<double,3> slack_gains = {1e5, 100, 1};
  arma::vec slack_limits = {2e-2, 0.2, 1.0};
  
  double time_limit = 0; // 2e-3;
  int max_iter = 12000;
  double abs_tol = 1e-3;
  double rel_tol = 1e-5;

  // --------  GMP - MPC  --------
  gmp_::GMP_MPC gmp_mpc(&gmp, N_horizon, pred_time_step, N_kernels, kernels_std_scaling, slack_gains);
  
  gmp_mpc.settings.max_iter = max_iter;
  gmp_mpc.settings.time_limit = time_limit;
  gmp_mpc.settings.abs_tol = abs_tol;
  gmp_mpc.settings.rel_tol = rel_tol;

  gmp_mpc.setObjCostGains(opt_pos, opt_vel);
  
  gmp_mpc.setPosLimits(pos_lim.col(0), pos_lim.col(1));
  gmp_mpc.setVelLimits(vel_lim.col(0), vel_lim.col(1));
  gmp_mpc.setAccelLimits(accel_lim.col(0), accel_lim.col(1));
  
  gmp_mpc.setPosSlackLimit(slack_limits[0]);
  gmp_mpc.setVelSlackLimit(slack_limits[1]);
  gmp_mpc.setAccelSlackLimit(slack_limits[2]);

  gmp_mpc.setInitialState(y, y_dot, y_ddot, 0, can_sys.sd_dot, 0);
  gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.sd_dot, 0, final_state_err_tol);

  auto can_sys_fun = [&can_sys](double s, double s_dot)
                      { return std::array<double,2>({s_dot, can_sys.getPhaseDDot(s, s_dot)}); };

  gmp_mpc.setCanonicalSystemFunction(can_sys_fun);

  for (auto &obst : obstacles) gmp_mpc.addEllipsoidObstacle(obst.c, obst.Sigma);

  gmp.setScaleMethod(gmp_::TrajScale::Ptr(new gmp_::TrajScale_Prop(n_dof)));
  gmp.setY0(y0);
  gmp.setGoal(yg);

//  t_start = tic;
  
  bool opt_fail = false;
  bool on_opt_fail_exit = true;
  
  arma::rowvec Time;
  arma::mat P_data;
  arma::mat dP_data;
  arma::mat ddP_data;
  arma::mat pos_slack_data;
  arma::mat vel_slack_data;
  arma::mat accel_slack_data;

  gmp_::GMP_MPC::Solution sol;

  double progress = 0.;
  double prog_step = 5;

  arma::rowvec s_data = arma::linspace<arma::rowvec>(0, 1, 200);
  arma::mat Pd_data(n_dof, s_data.size());
  for (int i=0; i<s_data.size(); i++) Pd_data.col(i) = gmp.getYd(s_data(i));
  std::vector<arma::mat> obst_curves;
  for (auto &e : obstacles) obst_curves.push_back(drawElipsoid2D(e.Sigma, e.c));

  OnlinePlot o_plot(gmp_mpc);
  o_plot.init(Pd_data, pos_lim.col(0), pos_lim.col(1), obst_curves, 5, 20);

  gmp_mpc.plot_callback = [&o_plot](const gmp_::GMP_MPC::Log &log){ o_plot(log); };

  program_pause();

  // -------  Simulation loop  --------
  while (true)
  {
    // update final state
    gmp_mpc.setFinalState(yg, O_ndof, O_ndof, 1, can_sys.sd_dot, 0, final_state_err_tol);

    progress = std::min(can_sys.s * 100., 100.);

    if (progress >= prog_step)
    {
      std::cout << "\33[1m\33[32mProgress: " << std::setprecision(2) << std::fixed << progress << " %\33[0m\n" << std::flush;
      prog_step += 5;
    }

    // Stopping criteria
    if (can_sys.s >= 1) break;

    // optimization
    if (!opt_fail)
    {
      // sovle
      sol = gmp_mpc.solve(can_sys.s, can_sys.s_dot);

      // check exit status
      if (sol.exit_flag)
      {
        PRINT_WARNING(sol.exit_msg);
        if (sol.exit_flag < 0) opt_fail = true;
        if (opt_fail && on_opt_fail_exit) break;
      }
    }

    // get optimal trajectory
    double s = can_sys.s;
    double s_dot = can_sys.s_dot;
    double s_ddot = can_sys.getPhaseDDot(s, s_dot);

    y = gmp_mpc.getYd(s); // sol.y;
    y_dot = gmp_mpc.getYdDot(s, s_dot); // sol.y_dot;
    y_ddot = gmp_mpc.getYdDDot(s, s_dot, s_ddot); // sol.y_ddot;

    // update initial state (optionally, if say a perturbation occurs)
    // gmp_mpc.setInitialState(y, y_dot, y_ddot, s, s_dot, s_ddot);

    // Log data
    Time = arma::join_horiz(Time, arma::vec({t}));
    P_data = arma::join_horiz(P_data, y);
    dP_data = arma::join_horiz(dP_data, y_dot);
    ddP_data = arma::join_horiz(ddP_data, y_ddot);

    if (!opt_fail)
    {
      pos_slack_data = arma::join_horiz(pos_slack_data, sol.pos_slack);
      vel_slack_data = arma::join_horiz(vel_slack_data, sol.vel_slack);
      accel_slack_data = arma::join_horiz(accel_slack_data, sol.accel_slack);
    }

    // Numerical integration
    can_sys.integrate(t, t+dt);
    t = t + dt;
  }
  
//  target_err = norm(P_data(:,end)-yg)
//  vel_err = norm(dP_data(:,end))
//  accel_err = norm(ddP_data(:,end))
//
//  max_slack_violations = [max(abs(pos_slack_data(:))), max(abs(vel_slack_data(:))), max(abs(accel_slack_data(:)))]
//
//  fprintf('===> GMP-MPC optimization finished! Elaps time: %f ms\n',toc(t_start)*1000);

  return Trajectory(Time, P_data, dP_data, ddP_data);
}





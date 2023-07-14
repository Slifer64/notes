#include <iostream>
#include <vector>
#include <cstring>
#include <functional>
#include <thread>
#include <chrono>

#include <armadillo>

#include <boost/numeric/odeint.hpp>

#include <plot_lib/qt_plot.h>

using namespace as64_;

using namespace boost::numeric::odeint;

/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;

const double gam = 0.15;

/* The rhs of x' = f(x) */
void harmonic_oscillator( const state_type &x , state_type &dxdt , const double /* t */ )
{
  dxdt[0] = x[1];
  dxdt[1] = -x[0] - gam*x[1];
}

struct push_back_state_and_time
{
  arma::mat &states_;
  arma::rowvec &times_;

  push_back_state_and_time( arma::mat &states , arma::rowvec &times )
  : states_( states ) , times_( times ) {}

  void operator()( const state_type &x , double t )
  {
    states_ = arma::join_horiz( states_, arma::vec(x) );
    times_ = arma::join_horiz( times_, arma::vec({t}) );
  }
};


int main(int argc, char **argv)
{
  pl_::QtPlot::init();

  state_type x(2);
  x[0] = 1.0; // start at x=1.0, p=0.0
  x[1] = 0.0;

  arma::mat  x_vec;
  arma::rowvec times;
  push_back_state_and_time observer(x_vec, times);

  // size_t steps = integrate( harmonic_oscillator , x , 0.0 , 10.0 , 0.1 , observer );

  typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
  typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
  controlled_stepper_type controlled_stepper;

  runge_kutta4< state_type > stepper;
  const double dt = 0.01;
  
  for( double t=0.0 ; t<10.0 ; t+= dt )
  {
    stepper.do_step( harmonic_oscillator , x , t , dt );
    integrate_adaptive( controlled_stepper , harmonic_oscillator , x , t , t+dt , dt);
    //integrate_adaptive( make_controlled< error_stepper_type >( 1.0e-10 , 1.0e-6 ) , harmonic_oscillator , x , 0.0 , 10.0 , 0.01 );
    observer(x, t);
  }
    

  // ===========  Plot  =========
  std::cerr << "times: " << times.n_rows << " x " << times.n_cols << "\n";
  std::cerr << "x_vec: " << x_vec.n_rows << " x " << x_vec.n_cols << "\n";

  pl_::Figure *fig_ = pl_::figure("", {500, 600});
  fig_->setAxes(2,1);
  for (int i=0; i<x.size(); i++)
  {
    pl_::Axes *ax = fig_->getAxes(i);
    ax->plot(times, x_vec.row(i), pl_::LineWidth_,2.0, pl_::Color_,pl_::RED);
    if (i == x.size()-1) ax->xlabel("Time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }

  pl_::Figure *fig2_ = pl_::figure("", {500, 600});
  fig2_->setAxes(2,1);
  for (int i=0; i<x.size(); i++)
  {
    pl_::Axes *ax = fig2_->getAxes(i);
    ax->plot(times, x_vec.row(i), pl_::LineWidth_,2.0, pl_::Color_,pl_::BLUE);
    if (i == x.size()-1) ax->xlabel("Time [s]", pl_::FontSize_,14);
    ax->drawnow();
  }

  // pl_::closeAll();
  
  std::string temp;
  std::getline(std::cin, temp);

  return 0;
}
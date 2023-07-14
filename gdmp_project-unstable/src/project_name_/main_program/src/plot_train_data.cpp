#include <exception>
#include <cstring>
#include <iostream>

#include <armadillo>

#include <ros/ros.h>
#include <ros/package.h>

#include <plot_lib/qt_plot.h>
#include <io_lib/file_io.h>
#include <io_lib/io_utils.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plot_train_data");

  // ============== Load data ===============
	arma::rowvec Time;
	arma::mat P_data;

  std::string path = ros::package::getPath("train_controller") + "/config/train_data.bin";
  std::cerr << "Loading logged data from:\n" << path << "\n";
	io_::FileIO fid(path, io_::FileIO::in);
	fid.read("Timed", Time);
	fid.read("Pd_data", P_data);
  fid.close();
  std::cerr << "[SUCCESS]\n";

  int n_data = Time.size();

  arma::mat dP_data(P_data.n_rows, P_data.n_cols);
  arma::rowvec dTime = arma::diff(Time);
  for (int i=0; i<P_data.n_rows; i++)
    dP_data.row(i) = arma::join_horiz(arma::diff(P_data.row(i))/dTime, arma::vec({0}));

  // =============== Plot data ==============
  std::cerr << "Plotting logged data...\n";
  pl_::QtPlot::init();

  pl_::Figure *fig;
  pl_::Axes *ax;

  fig = pl_::figure("", {500, 600});
  fig->setAxes(2,1);
  ax = fig->getAxes(0);
  ax->hold(true);
  // ax->xlabel("time [s]");
  for (int i=0;i<3;i++) pl_::Graph *graph = ax->plot(Time, P_data.row(i), pl_::LineWidth_,2.0);
  ax->title("Cartesian Position", pl_::FontSize_,16, pl_::FontWeight_,pl_::DemiBold);
  ax->ylabel("Position [m]", pl_::FontSize_,14);
  ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
  ax->drawnow();

  ax = fig->getAxes(1);
  ax->hold(true);
  // ax->xlabel("time [s]");
  for (int i=0;i<3;i++) ax->plot(Time, dP_data.row(i), pl_::LineWidth_,2.0);
  // ax->title("Cartesian Velocity");
  ax->ylabel("Velocity [m/s]", pl_::FontSize_,14);
  ax->xlabel("Time [s]", pl_::FontSize_,14);
  ax->legend({"x", "y", "z"}, pl_::FontSize_,15);
  ax->drawnow();

	as64_::io_::PRINT_INFO_MSG("Press [enter] to exit...\n");
  std::string dummy;
	std::getline(std::cin, dummy, '\n');

	as64_::io_::PRINT_CONFIRM_MSG("Finished!!!\n");


  return 0;
}



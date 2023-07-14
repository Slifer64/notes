#ifndef $_PROJECT_384$_GMP_MPC_CONTROLLER_EXPERIMENT_H
#define $_PROJECT_384$_GMP_MPC_CONTROLLER_EXPERIMENT_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <list>
#include <memory>
#include <mutex>
#include <functional>
#include <armadillo>
#include <Eigen/Dense>

#include <gmp_lib/GMP/GMP_MPC.h>
#include <gmp_lib/GMP/GMP_Opt.h>
#include <gmp_lib/CanonicalSystem/CanonicalSystem.h>

#include <io_lib/file_io.h>

#include <gmp_lib/io/gmp_io.h>
#include <yaml-cpp/yaml.h>
// #include <plot_lib/qt_plot.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <apriltag_ros/apriltag_listener.h>

using namespace as64_;

namespace gmp_mpc_ctrl_
{

class Experiment
{
public:

  Experiment() {}

  virtual bool init() = 0;

  virtual void execute() = 0;


protected:

  

};

} // namespace gmp_mpc_ctrl_

#endif // $_PROJECT_384$_GMP_MPC_CONTROLLER_EXPERIMENT_H

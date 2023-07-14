#include <robot_wrapper/utils/inv_kinematics_interface.h>
#include <robot_wrapper/utils/xml_parser.h>

#include <ros/package.h>

namespace rw_
{

#define InvKinInterface_fun_ std::string("[InvKinematicsInterface::") + __func__ + "]: "

InvKinematicsInterface::InvKinematicsInterface()
{
}

InvKinematicsInterface::~InvKinematicsInterface()
{

}

void InvKinematicsInterface::initInvKinematicsInterface()
{
  std::string params_filename = ros::package::getPath("robot_wrapper") + "/config/params.yaml";
  loadParams(params_filename);
}

void InvKinematicsInterface::loadParams(const std::string &filename)
{
  rw_::XmlParser parser(filename);

  std::string load_fail_msg = InvKinInterface_fun_ + "Failed to load param ";

  // =======  check whether to use singular value filtering for pinv  =======
  bool use_svf_ = false;
  if (parser.getParam("use_svf", use_svf_) && use_svf_)
  {
    double sigma_min, shape_f;
    if (!parser.getParam("sigma_min", sigma_min)) throw std::runtime_error(load_fail_msg + "\"sigma_min\"...");
    if (!parser.getParam("shape_f", shape_f)) throw std::runtime_error(load_fail_msg + "\"shape_f\"...");
    this->setJacobInvFilter(sigma_min, shape_f);
  }
  this->use_svf = use_svf_;
}

void InvKinematicsInterface::setJacobInvFilter(double sigma_min, double shape_f)
{
  svf.reset(new robo_::SingularValueFilter(sigma_min, shape_f));
  use_svf = true;
}

arma::mat InvKinematicsInterface::getInvJacobian() const
{
  if (use_svf) return svf->pinv(getJacobian());
  else return arma::pinv(getJacobian());
}


} // namespace rw_

#include <online_adapt_controller/utils/tag_reader.h>

#include <std_msgs/Int16.h>

#include <math_lib/math_lib.h>

using namespace as64_;

namespace online_adapt_
{

#define TagReader_fun_ std::string("[online_adapt_::TagReader::") + __func__ + "]: "

TagReader::TagReader()
{
  tag_id = 0;
  use_fixed = false;
  fixed_pos = {0, 0, 0};
  fixed_quat = {1, 0, 0, 0};
  max_times_tag_not_detected = 1;
  
  times_tag_not_detected = 0;
}

TagReader::~TagReader()
{}

int TagReader::getPose(arma::vec *P, arma::vec *Q)
{
  bool success = getPoseHelper(P, Q);
  times_tag_not_detected = success ? 0 : times_tag_not_detected+1;

  if (times_tag_not_detected == 0) return 1; // found target
  else if (times_tag_not_detected < max_times_tag_not_detected) return 2; // didn't find target, but iterations not exceeded
  else return 0; // iterations to find target exceeded
}

bool TagReader::getPoseHelper(arma::vec *P, arma::vec *Q)
{
  if (!get_tag_fun)
  {
    std::cerr << "\33[1m\33[33m" << TagReader_fun_ << "You must call \"setTagReadFunction\" first!\n\33[0m" << std::flush;
    return false;
  }

  if (use_fixed)
  {
    *P = fixed_pos;
    *Q = fixed_quat;
    return true;
  }

  bool success_ =  this->get_tag_fun(P, Q);
  if (!success_) return false;

  // if (tag_id == 5)
  // {
  //   std::cerr << "offset.pos = " << offset.pos.t() << "\n"; 
  //   std::cerr << "offset.rpy = " << offset.rpy.t() << "\n"; 
  // }
  

  arma::mat R = math_::quat2rotm(*Q);
  *P += R*offset.pos;
  Eigen::Quaterniond quat = Eigen::AngleAxisd(offset.rpy[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(offset.rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(offset.rpy[2], Eigen::Vector3d::UnitZ());
  arma::vec Q_offset = {quat.w(), quat.x(), quat.y(), quat.z()};
  
  *Q = math_::quatProd(*Q, Q_offset);
  return true;

  // return this->get_tag_fun(P, Q);
}

int TagReader::tagId() const
{
  return tag_id;
}

void TagReader::setTagReadFunction(std::function<bool(arma::vec *, arma::vec *)> get_tag_fun)
{
  this->get_tag_fun = get_tag_fun;
}

ViaPoints TagReader::getViaPoints(const arma::vec &Pg, const arma::vec &Qg) const
{
  ViaPoints via_poitns;

  via_poitns.pos.reserve(vp.size());
  via_poitns.quat.reserve(vp.size());

  arma::mat Rg = math_::quat2rotm(Qg);
  for (auto &vp_i : vp)
  {
    via_poitns.pos.push_back(Pg + Rg*vp_i);
    via_poitns.quat.push_back(Qg);
  }
  return via_poitns;
}

void TagReader::loadParams(const std::string &filename, const std::string params_prefix)
{
  try
  {
    YAML::Node nh = YAML::LoadFile(filename);

    if (params_prefix.empty()) loadParams(nh);
    else
    {
      YAML::Node params_nh;
      if ( !YAML::getParam(nh, params_prefix, params_nh) ) throw std::runtime_error("Failed to load param \"" + params_prefix + "\"...");
      loadParams(params_nh);
    }
  }
  catch (std::exception &e)
  { throw std::runtime_error(TagReader_fun_ + e.what()); }
}

void TagReader::loadParams(const YAML::Node &n)
{
  if ( !YAML::getParam(n, "tag_id", tag_id) ) throw std::runtime_error(TagReader_fun_ + "Failed to load param \"tag_id\"...");
  if ( !YAML::getParam(n, "max_times_tag_not_detected", max_times_tag_not_detected) ) max_times_tag_not_detected = -1; // throw std::runtime_error(TagReader_fun_ + "Failed to load param \"max_times_tag_not_detected\"...");
  if ( !YAML::getParam(n, "use_fixed", use_fixed) ) use_fixed = false; // throw std::runtime_error(TagReader_fun_ + "Failed to load param \"use_fixed\"...");
  if ( use_fixed && !YAML::getParam(n, "fixed_pos", fixed_pos) ) throw std::runtime_error(TagReader_fun_ + "Failed to load param \"fixed_pos\"...");
  if ( use_fixed && !YAML::getParam(n, "fixed_quat", fixed_quat) ) throw std::runtime_error(TagReader_fun_ + "Failed to load param \"fixed_quat\"...");
  fixed_quat = fixed_quat / arma::norm(fixed_quat);

  offset.pos = {0, 0, 0};
  offset.rpy = {0, 0, 0};
  YAML::Node offset_n;
  if (YAML::getParam(n, "offset", offset_n))
  {
    YAML::getParam(offset_n, "pos", offset.pos);
    YAML::getParam(offset_n, "rpy", offset.rpy);
  }

  vp.clear();
  arma::mat temp_vp;
  if (YAML::getParam(n, "vp", temp_vp))
  {
    temp_vp = temp_vp.t(); // one via-point per column
    for (int j=0; j<temp_vp.n_cols; j++) vp.push_back(temp_vp.col(j));
  }

}

} // namespace online_adapt_

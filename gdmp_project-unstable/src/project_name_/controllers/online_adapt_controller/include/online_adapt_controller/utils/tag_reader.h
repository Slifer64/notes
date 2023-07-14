#ifndef $_PROJECT_384$_ONLINE_ADAPT_TAG_READER_H
#define $_PROJECT_384$_ONLINE_ADAPT_TAG_READER_H

#include <vector>
#include <cstring>
#include <tuple>

#include <armadillo>

#include <yaml-cpp/yaml.h>

namespace online_adapt_
{

struct ViaPoints
{
  std::vector<arma::vec> pos;
  std::vector<arma::vec> quat;
};

class TagReader
{
public:

  TagReader();
  ~TagReader();

  void setTagReadFunction(std::function<bool(arma::vec *, arma::vec *)> get_tag_fun);

  int getPose(arma::vec *P, arma::vec *Q);

  int tagId() const;

  void loadParams(const std::string &filename, const std::string params_prefix="");
  void loadParams(const YAML::Node &n);

  bool getPoseHelper(arma::vec *P, arma::vec *Q);

  ViaPoints getViaPoints(const arma::vec &Pg, const arma::vec &Qg) const;

private:

  std::function<bool(arma::vec *, arma::vec *)> get_tag_fun;

  int times_tag_not_detected;

  // params
  int tag_id;
  int max_times_tag_not_detected;
  bool use_fixed;
  arma::vec fixed_pos;
  arma::vec fixed_quat;

  std::vector<arma::vec> vp;

  struct{
    arma::vec pos;
    arma::vec rpy;
  } offset;
  
};


} // namespace online_adapt_

#endif // $_PROJECT_384$_ONLINE_ADAPT_TAG_READER_H

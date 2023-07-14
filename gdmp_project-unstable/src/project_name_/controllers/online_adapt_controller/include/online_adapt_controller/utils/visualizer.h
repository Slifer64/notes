#ifndef $_PROJECT_384$_ONLINE_ADAPT_VISUALIZER_H
#define $_PROJECT_384$_ONLINE_ADAPT_VISUALIZER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <armadillo>

#include <rviz_lib/orient_path_publisher.h>
#include <rviz_lib/frame_publisher.h>
#include <yaml-cpp/yaml.h>

using namespace as64_;

namespace online_adapt_
{

class VizParams
{
public:

  void loadParams(const YAML::Node &n, const std::string params_prefix="");

  bool enabled;
  bool pub_path;
  bool pub_frame;
  double frame_dist;
};

class Visualizer
{
public:

  Visualizer(const std::string &marker_array_topic, const std::string &base_frame);
  ~Visualizer();

  void viewTag(const std::string &name, bool set, std::function<bool(arma::vec *, arma::vec *Q)> get_tag_pose_fun=NULL, unsigned pub_rate=100);

  void start();
  void stop();

  void enable(bool set) { pub_on = set; }

  void setRobotState(const arma::vec &P, const arma::vec &Q);
  void setRefState(const arma::vec &P_ref, const arma::vec &Q_ref);

  void setViapoint(const arma::vec &P, const arma::vec &Q);

  void setPlaceViapoints(const std::vector<arma::vec> &P, const std::vector<arma::vec> &Q);
  void setObstViapoints(const std::vector<arma::vec> &P, const std::vector<arma::vec> &Q);

  void publishFrames(bool set);

  void showObstacle(const arma::vec &center, const arma::vec &orient, double x_len, double y_len, double z_len);
  void clearObstacle();

  void loadParams(const std::string &filename, const std::string params_prefix="");
  void loadParams(const YAML::Node &n);

  void clearViapoints();
  void clearRobotPath();
  void clearRefPath();
  void clearPlaceViapoints();
  void clearObstViapoints();

private:

  bool pub_on = true;

  std::map<std::string, std::unique_ptr<rviz_::FramePublisher>> tag_pub_map;

  std::unique_ptr<rviz_::OrientPathPublisher> robot_path_pub;
  std::unique_ptr<rviz_::OrientPathPublisher> dmp_path_pub;
  std::unique_ptr<rviz_::OrientPathPublisher> vp_pub;

  std::unique_ptr<rviz_::RvizMarkerPublisher> rviz_pub;
  std::mutex rviz_mtx;

  // ------ params ---------
  VizParams robot_params;
  VizParams dmp_params;
  VizParams vp_params;

  unsigned pub_rate;

  // ----------------------

  std::string marker_array_topic;
  std::string base_frame;

  std::thread pub_thread;
  bool run_;
};


} // namespace online_adapt_




#endif // $_PROJECT_384$_ONLINE_ADAPT_VISUALIZER_H

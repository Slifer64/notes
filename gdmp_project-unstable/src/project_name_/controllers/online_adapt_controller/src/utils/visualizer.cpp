#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <online_adapt_controller/utils/visualizer.h>

#include <rviz_lib/orient_path_publisher.h>
#include <yaml-cpp/yaml.h>

using namespace as64_;

namespace online_adapt_
{

#define Visualizer_fun_ std::string("[online_adapt_::Visualizer::") + __func__ + "]: "


void VizParams::loadParams(const YAML::Node &n, const std::string params_prefix)
{
  YAML::Node nh;
  if (!params_prefix.empty())
  {
    if ( !YAML::getParam(n, params_prefix, nh) ) throw std::runtime_error("Failed to load param \"" + params_prefix + "\"...");
  }
  else nh = n;

  if ( !YAML::getParam(nh, "enabled", enabled) ) throw std::runtime_error("Failed to load param \"enabled\"...");
  if ( !YAML::getParam(nh, "pub_path", pub_path) ) pub_path = true;
  if ( !YAML::getParam(nh, "pub_frame", pub_frame) ) pub_frame = true;
  if ( !YAML::getParam(nh, "frame_dist", frame_dist) ) frame_dist = 0.1;
}


Visualizer::Visualizer(const std::string &marker_array_topic, const std::string &base_frame)
{
  this->marker_array_topic = marker_array_topic;
  this->base_frame = base_frame;

  robot_path_pub.reset(new rviz_::OrientPathPublisher(marker_array_topic, base_frame, "robot_path"));
  dmp_path_pub.reset(new rviz_::OrientPathPublisher(marker_array_topic, base_frame, "dmp_path"));
  vp_pub.reset(new rviz_::OrientPathPublisher(marker_array_topic, base_frame, "viapoint_path"));
  rviz_pub.reset(new rviz_::RvizMarkerPublisher(marker_array_topic, base_frame));

  rviz_pub->enable_topic_not_exist_warnings = false;
}

Visualizer::~Visualizer()
{
  stop();
  for (auto it=tag_pub_map.begin(); it!= tag_pub_map.end(); it++) viewTag(it->first, false);
}

void Visualizer::start()
{
  if (robot_params.enabled)
  {
    // robot_path_pub->start(pub_rate);
    robot_path_pub->init();
    robot_path_pub->frame_scale = 0.9;
    robot_path_pub->line_width = 0.02;
    robot_path_pub->publishIntermediateFrames(robot_params.pub_frame, robot_params.frame_dist);
    robot_path_pub->publishPath(robot_params.pub_path);
  }
  
  if (dmp_params.enabled)
  {
    // dmp_path_pub->start(pub_rate);
    dmp_path_pub->init();
    dmp_path_pub->frame_scale = 0.9;
    dmp_path_pub->line_width = 0.02;
    dmp_path_pub->publishIntermediateFrames(dmp_params.pub_frame, dmp_params.frame_dist);
    dmp_path_pub->publishPath(dmp_params.pub_path);
    dmp_path_pub->color = rviz_::Color(1.0, 0.5, 0.0, 0.3);
    dmp_path_pub->frame_x_color = rviz_::Color(1, 0, 0, 0.25);
    dmp_path_pub->frame_y_color = rviz_::Color(0, 1, 0, 0.25);
    dmp_path_pub->frame_z_color = rviz_::Color(0, 0, 1, 0.25);
  }

  if (vp_params.enabled)
  {
    vp_pub->init();
    vp_pub->frame_scale = 1.1;
    vp_pub->line_width = 0.02;
    vp_pub->publishIntermediateFrames(vp_params.pub_frame, vp_params.frame_dist);
    vp_pub->publishPath(vp_params.pub_path);
    vp_pub->color = rviz_::Color(1.0, 0.5, 0.0, 0.3);
    vp_pub->frame_x_color = rviz_::Color(1, 0, 0.5, 0.7);
    vp_pub->frame_y_color = rviz_::Color(0.4, 1, 0.4, 0.7);
    vp_pub->frame_z_color = rviz_::Color(0, 0.5, 1, 0.7);
  }
  
  run_ = true;
  pub_thread = std::thread([this]()
  {
    while (run_)
    {
      if (pub_on)
      {
        robot_path_pub->publishNow();
        dmp_path_pub->publishNow();
        vp_pub->publishNow();
      
        {
          std::unique_lock<std::mutex> lck(rviz_mtx);
          rviz_pub->drawnow();
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(pub_rate));
    }
  });
}

void Visualizer::stop()
{
  run_ = false;
  if (pub_thread.joinable()) pub_thread.join();

  // robot_path_pub->stop();
  clearRobotPath();

  // dmp_path_pub->stop();
  clearRefPath();

  clearViapoints();

  clearPlaceViapoints();

  clearObstViapoints();

  clearObstacle();
}

void Visualizer::clearViapoints()
{
  vp_pub->clearPath();
}

void Visualizer::clearRobotPath()
{
  robot_path_pub->clearPath();
}

void Visualizer::clearRefPath()
{
  dmp_path_pub->clearPath();
}

void Visualizer::clearPlaceViapoints()
{
  std::unique_lock<std::mutex> lck(rviz_mtx);
  rviz_pub->deleteMarkers("place-vp");
  rviz_pub->drawnow();
}

void Visualizer::clearObstViapoints()
{
  std::unique_lock<std::mutex> lck(rviz_mtx);
  rviz_pub->deleteMarkers("obst-vp");
  rviz_pub->drawnow();
}

void Visualizer::setRobotState(const arma::vec &P, const arma::vec &Q)
{
  if (robot_params.enabled) robot_path_pub->addPoint(P, Q);
}

void Visualizer::setRefState(const arma::vec &P_ref, const arma::vec &Q_ref)
{
  if (dmp_params.enabled) dmp_path_pub->addPoint(P_ref, Q_ref);
}

void Visualizer::setViapoint(const arma::vec &P, const arma::vec &Q)
{
  if (vp_params.enabled) vp_pub->addPoint(P, Q);
}

void Visualizer::setPlaceViapoints(const std::vector<arma::vec> &P, const std::vector<arma::vec> &Q)
{
  static std::vector<rviz_::Color> axes_color = { rviz_::Color(1, 0, 0.5, 0.7), rviz_::Color(0.4, 1, 0.4, 0.7), rviz_::Color(0, 0.5, 1, 0.7)};

  std::unique_lock<std::mutex> lck(rviz_mtx);
  rviz_pub->deleteMarkers("place-vp");
  for (int i=0; i<P.size(); i++)
  {
    Eigen::Vector3d pos(P[i](0), P[i](1), P[i](2));
    Eigen::Quaterniond quat(Q[i](0), Q[i](1), Q[i](2), Q[i](3));
    rviz_pub->publishFrame(pos, quat, 1.1, axes_color, "place-vp");
  }
}

void Visualizer::setObstViapoints(const std::vector<arma::vec> &P, const std::vector<arma::vec> &Q)
{
  static std::vector<rviz_::Color> axes_color = { rviz_::Color(0.7, 0, 0.2, 0.7), rviz_::Color(0.4, 0.8, 0.4, 0.7), rviz_::Color(0, 0.2, 0.7, 0.7)};

  std::unique_lock<std::mutex> lck(rviz_mtx);
  rviz_pub->deleteMarkers("obst-vp");
  for (int i=0; i<P.size(); i++)
  {
    Eigen::Vector3d pos(P[i](0), P[i](1), P[i](2));
    Eigen::Quaterniond quat(Q[i](0), Q[i](1), Q[i](2), Q[i](3));
    rviz_pub->publishFrame(pos, quat, 1.1, axes_color, "obst-vp");
  }
}

void Visualizer::showObstacle(const arma::vec &center, const arma::vec &orient, double x_len, double y_len, double z_len)
{
  clearObstacle();
  std::unique_lock<std::mutex> lck(rviz_mtx);
  rviz_pub->publishCuboid(Eigen::Vector3d(center(0), center(1), center(2)), Eigen::Quaterniond(orient(0), orient(1), orient(2), orient(3)), x_len, y_len, z_len, rviz_::Color(1, 0, 0, 0.2), "obst-bounds");
  rviz_pub->drawnow();
}

void Visualizer::clearObstacle()
{
  std::unique_lock<std::mutex> lck(rviz_mtx);
  rviz_pub->deleteMarkers("obst-bounds");
  rviz_pub->drawnow();
}


void Visualizer::publishFrames(bool set)
{
  robot_path_pub->publishIntermediateFrames(set);
  dmp_path_pub->publishIntermediateFrames(set);
}

void Visualizer::viewTag(const std::string &name, bool set, std::function<bool(arma::vec *, arma::vec *Q)> get_tag_pose_fun, unsigned pub_rate)
{
  auto it = tag_pub_map.find(name);

  if (!set && it==tag_pub_map.end())
  {
    std::cerr << "\33[1m\33[33m[Visualizer::viewTag]: Requested to stop tag view of unregistered tag name \"" + name + "\"!\n\33[0m" << std::flush;
    return;
  }

  if (set && it!=tag_pub_map.end())
  {
    std::cerr << "\33[1m\33[33m[Visualizer::viewTag]: Requested to start tag view of already registered tag name \"" + name + "\"!\n\33[0m" << std::flush;
    return;
  }

  if (set)
  {
    tag_pub_map[name] = std::unique_ptr<rviz_::FramePublisher>(new rviz_::FramePublisher(get_tag_pose_fun, marker_array_topic, base_frame, name));
    auto tag_pub = tag_pub_map[name].get();
    tag_pub->setAxesColor(rviz_::Color(1, 0, 0, 0.6), rviz_::Color(0, 1, 0, 0.6), rviz_::Color(0, 0, 1, 0.6));
    tag_pub->setAxesScale(1.3);
    tag_pub->start(pub_rate);
  }
  else
  {
    it->second->stop();
    tag_pub_map.erase(it);
  }
}

void Visualizer::loadParams(const std::string &filename, const std::string params_prefix)
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
  { throw std::runtime_error(Visualizer_fun_ + e.what()); }
}

void Visualizer::loadParams(const YAML::Node &n)
{
  robot_params.loadParams(n, "robot");
  dmp_params.loadParams(n, "dmp");
  vp_params.loadParams(n, "viapoint");

  if ( !YAML::getParam(n, "pub_rate", pub_rate) ) throw std::runtime_error(Visualizer_fun_ + "Failed to load param \"pub_rate\"...");
}



} // namespace online_adapt_


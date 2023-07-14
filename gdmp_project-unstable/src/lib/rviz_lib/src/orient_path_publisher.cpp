#include <rviz_lib/orient_path_publisher.h>

namespace as64_
{

namespace rviz_
{

OrientPathPublisher::OrientPathPublisher(const std::string &marker_array_topic, const std::string &base_frame, const std::string &path_name)
{
  this->path_name = path_name;
  rviz_pub.reset(new rviz_::RvizMarkerPublisher(marker_array_topic, base_frame));
  rviz_pub->enable_topic_not_exist_warnings = false;
  exist_published_data = false;

  publishIntermediateFrames(true);
  publishPath(true);
}

OrientPathPublisher::~OrientPathPublisher()
{
  stop();
}

void OrientPathPublisher::start(unsigned pub_cycle_ms)
{
  run_ = true;
  init();
  this->pub_cycle_ms = pub_cycle_ms;
  pub_path_thread = std::thread([this](){ publishPathThread(); });
}

void OrientPathPublisher::stop()
{
  run_ = false;
  if (pub_path_thread.joinable()) pub_path_thread.join();
}

void OrientPathPublisher::clearPath()
{
  if (!exist_published_data) return;
  // rviz_pub->deleteMarkers(this->path_name);
  rviz_pub->deleteAllMarkers();
  rviz_pub->drawnow();
  exist_published_data = false;
}

void OrientPathPublisher::publishIntermediateFrames(bool set, double dist)
{
  publish_frame = set;
  orient_frames_dist = dist;
}

void OrientPathPublisher::publishPath(bool set)
{
  publish_path = set;
}

void OrientPathPublisher::init()
{
  dist = 0;
  is_path_initialized = false;
}

void OrientPathPublisher::publishNow()
{
  if (!is_path_initialized) return;

  Eigen::Vector3d p_prev, p_current;
  Eigen::Quaterniond quat;
  {
    std::unique_lock<std::mutex> lck(this->current_pos_mtx);
    p_prev = Eigen::Vector3d(prev_pos(0), prev_pos(1), prev_pos(2));
    p_current = Eigen::Vector3d(current_pos(0), current_pos(1), current_pos(2));
    quat = Eigen::Quaterniond(current_quat(0), current_quat(1), current_quat(2), current_quat(3));
  }
  this->prev_pos = this->current_pos;

  dist += (p_current - p_prev).norm();

  rviz_pub->deleteMarkers(this->path_name + "_current_frame");
  std::string frame_suffix = "_current_frame";
  if (publish_frame && dist > orient_frames_dist)
  {
    frame_suffix = "_orient_frame";
    dist = 0;
  }
  rviz_pub->publishFrame(p_current, quat, frame_scale, {frame_x_color, frame_y_color, frame_z_color}, this->path_name + frame_suffix);

  if (publish_path) rviz_pub->publishCylinder(p_prev, p_current, this->line_width/2, this->color, this->path_name);
  rviz_pub->drawnow();
  exist_published_data = true;
}

void OrientPathPublisher::publishPathThread()
{
  while (run_)
  {
    publishNow();
    std::this_thread::sleep_for(std::chrono::milliseconds(this->pub_cycle_ms));
  }
}

void OrientPathPublisher::addPoint(const arma::vec &pos, const arma::vec &quat)
{
  std::unique_lock<std::mutex> lck(this->current_pos_mtx);

  if (is_path_initialized)
  {
    this->current_pos = pos;
    this->current_quat = quat;
  }
  else
  {
    this->prev_pos = this->current_pos = pos;
    this->prev_quat = this->current_quat = quat;
  }

  is_path_initialized = true;
}

} // rviz_

} // as64_


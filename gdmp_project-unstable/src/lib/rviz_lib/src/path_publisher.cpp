#include <rviz_lib/path_publisher.h>

namespace as64_
{

namespace rviz_
{

PathPublisher::PathPublisher(const std::string &marker_array_topic, const std::string &base_frame, const std::string &path_name)
{
  this->path_name = path_name;
  rviz_pub.reset(new rviz_::RvizMarkerPublisher(marker_array_topic, base_frame));
  rviz_pub->enable_topic_not_exist_warnings = false;
  exist_published_data = false;
}

PathPublisher::~PathPublisher()
{
  stop();
}

void PathPublisher::start(unsigned pub_cycle_ms)
{
  run_ = true;
  is_path_initialized = false;
  this->pub_cycle_ms = pub_cycle_ms;
  pub_path_thread = std::thread([this](){ publishPath(); });
}

void PathPublisher::stop()
{
  run_ = false;
  if (pub_path_thread.joinable()) pub_path_thread.join();
}

void PathPublisher::clearPath()
{
  if (!exist_published_data) return;
  rviz_pub->deleteMarkers(this->path_name);
  rviz_pub->drawnow();
  exist_published_data = false;
}

void PathPublisher::publishPath()
{
  while (run_)
  {
    if (is_path_initialized)
    {
      Eigen::Vector3d p1, p2;
      {
        std::unique_lock<std::mutex> lck(this->current_pos_mtx);
        p1 = Eigen::Vector3d(prev_pos(0), prev_pos(1), prev_pos(2));
        p2 = Eigen::Vector3d(current_pos(0), current_pos(1), current_pos(2));
      }
      this->prev_pos = this->current_pos;
      rviz_pub->publishCylinder(p1, p2, this->line_width/2, this->color, this->path_name);
      rviz_pub->drawnow();
      exist_published_data = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(this->pub_cycle_ms));
  }
}

void PathPublisher::addPoint(const arma::vec &pos)
{
  std::unique_lock<std::mutex> lck(this->current_pos_mtx);

  if (is_path_initialized)
  {
    this->current_pos = pos;
  }
  else this->prev_pos = this->current_pos = pos;

  is_path_initialized = true;
}

} // rviz_

} // as64_


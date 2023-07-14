#include <rviz_lib/frame_publisher.h>

namespace as64_
{

namespace rviz_
{

FramePublisher::FramePublisher(std::function<bool(arma::vec *, arma::vec *)> getPose, const std::string &marker_array_topic, const std::string base_link, const std::string &ns)
{
  rviz_pub.reset( new rviz_::RvizMarkerPublisher(marker_array_topic, base_link) );
  rviz_pub->enable_topic_not_exist_warnings = false;
  this->ns = ns;
  this->get_pose_fun_ = getPose;

  setAxesColor(rviz_::Color::RED, rviz_::Color::GREEN, rviz_::Color::BLUE);
  setAxesScale(1.2);
}

void FramePublisher::start(unsigned pub_rate_ms)
{
  pub_rate_ms_ = pub_rate_ms;
  setPublish(true);

  pub_thread = std::thread([this]()
  {
    arma::vec pose;
    while (publish_)
    {
      arma::vec pos, quat;
      bool success = get_pose_fun_(&pos, &quat);
      rviz_pub->deleteMarkers(ns);
      if (success)
        rviz_pub->publishFrame(Eigen::Vector3d(pos(0), pos(1), pos(2)), Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)), ax_scale, ax_colors, ns);
      rviz_pub->drawnow();
      std::this_thread::sleep_for(std::chrono::milliseconds(pub_rate_ms_));
    }
  });
}

void FramePublisher::stop()
{
  setPublish(false);
  if (pub_thread.joinable()) pub_thread.join();
  
  rviz_pub->deleteMarkers(ns);
  rviz_pub->drawnow();
}

void FramePublisher::setPublish(bool set)
{
  std::unique_lock<std::mutex> lck(pub_mutex);
  publish_ = set;
}

void FramePublisher::setAxesColor(const rviz_::Color &x_color, const rviz_::Color &y_color, const rviz_::Color &z_color)
{
  ax_colors = {x_color, y_color, z_color};
}

void FramePublisher::setAxesScale(double scale)
{
  ax_scale = scale;
}

} // rviz_

} // as64_


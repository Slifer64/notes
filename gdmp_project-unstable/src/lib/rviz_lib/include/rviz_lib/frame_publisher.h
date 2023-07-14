
#ifndef AS64_FRAME_PUBLISHER_H
#define AS64_FRAME_PUBLISHER_H

#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>

#include <rviz_lib/rviz_marker_publisher.h>

#include <armadillo>

namespace as64_
{

namespace rviz_
{

class FramePublisher
{
public:
  FramePublisher(std::function<bool(arma::vec *, arma::vec *)> getPose, const std::string &marker_array_topic, const std::string base_link, const std::string &ns);

  void start(unsigned pub_rate_ms = 50);

  void stop();

  void setPublish(bool set);

  void setAxesColor(const rviz_::Color &x_color, const rviz_::Color &y_color, const rviz_::Color &z_color);

  void setAxesScale(double scale);

private:

  std::unique_ptr<rviz_::RvizMarkerPublisher> rviz_pub;
  std::string ns;

  std::function<bool(arma::vec *, arma::vec *)> get_pose_fun_;

  unsigned pub_rate_ms_;
  bool publish_;
  std::mutex pub_mutex;

  std::vector<rviz_::Color> ax_colors;
  double ax_scale;

  std::thread pub_thread;
};



} // namespace rviz_

} // namespace as64_

#endif // AS64_FRAME_PUBLISHER_H

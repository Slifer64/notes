#ifndef AS64_RVIZ_LIB_PUBLISH_PATH_H
#define AS64_RVIZ_LIB_PUBLISH_PATH_H

#include <thread>
#include <mutex>
#include <rviz_lib/rviz_marker_publisher.h>

namespace as64_
{

namespace rviz_
{

class PathPublisher
{
public:

  PathPublisher(const std::string &marker_array_topic, const std::string &base_frame, const std::string &path_name);
  ~PathPublisher();

  void addPoint(const arma::vec &pos);
  void start(unsigned pub_cycle_ms = 50);
  void stop();
  void clearPath();

  rviz_::Color color = rviz_::Color::BROWN;
  double line_width = 0.02;

private:
  std::string path_name;
  unsigned pub_cycle_ms;

  bool exist_published_data;

  std::thread pub_path_thread;
  bool is_path_initialized = false;
  bool run_;
  void publishPath();
  
  std::mutex current_pos_mtx;
  arma::vec current_pos;
  arma::vec prev_pos;

  std::shared_ptr<rviz_::RvizMarkerPublisher> rviz_pub;
};

} // rviz_

} // as64_


#endif // AS64_RVIZ_LIB_PUBLISH_PATH_H

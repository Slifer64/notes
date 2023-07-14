#ifndef AS64_RVIZ_LIB_PUBLISH_ORIENT_PATH_H
#define AS64_RVIZ_LIB_PUBLISH_ORIENT_PATH_H

#include <thread>
#include <mutex>
#include <rviz_lib/rviz_marker_publisher.h>

namespace as64_
{

namespace rviz_
{

class OrientPathPublisher
{
public:

  OrientPathPublisher(const std::string &marker_array_topic, const std::string &base_frame, const std::string &path_name);
  ~OrientPathPublisher();

  void publishIntermediateFrames(bool set, double dist = 0.1);
  void publishPath(bool set);

  void addPoint(const arma::vec &pos, const arma::vec &quat);
  void clearPath();

  void init();
  void publishNow();

  // To run as a separate thread
  void start(unsigned pub_cycle_ms = 50);
  void stop();
  

  rviz_::Color color = rviz_::Color::BROWN;
  double line_width = 0.02;

  double frame_scale = 1.0;

  rviz_::Color frame_x_color = rviz_::Color::RED;
  rviz_::Color frame_y_color = rviz_::Color::GREEN;
  rviz_::Color frame_z_color = rviz_::Color::BLUE;

private:

  double dist; // current dist so far
  double orient_frames_dist;

  std::string path_name;

  bool exist_published_data;
  bool is_path_initialized = false;

  std::thread pub_path_thread;
  unsigned pub_cycle_ms;
  bool run_;
  void publishPathThread();
  
  std::mutex current_pos_mtx;
  arma::vec current_pos;
  arma::vec prev_pos;
  arma::vec current_quat;
  arma::vec prev_quat;

  bool publish_frame;
  bool publish_path;

  std::shared_ptr<rviz_::RvizMarkerPublisher> rviz_pub;
};

} // rviz_

} // as64_


#endif // AS64_RVIZ_LIB_PUBLISH_ORIENT_PATH_H

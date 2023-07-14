
#ifndef AS64_ROS_LIB_TRANSFORM_READER_H
#define AS64_ROS_LIB_TRANSFORM_READER_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <mutex>

namespace as64_
{
  
namespace ros_lib_
{

class TransformReader
{
public:
  TransformReader(const std::string &read_from_topic)
  {
    pos_ = {0, 0, 0};
    quat_ = {1, 0, 0, 0};

    is_updated = false;

    sub = nh.subscribe(read_from_topic, 1, &TransformReader::readFromTopicCallback, this);
  }

  arma::vec pos() const
  {
    std::unique_lock<std::mutex> lck(mtx);
    return pos_;
  }
  
  arma::vec quat() const
  {
    std::unique_lock<std::mutex> lck(mtx);
    return quat_;
  }

  bool isUpdated() const { return is_updated; }

protected:

  void readFromTopicCallback(const geometry_msgs::TransformStamped &msg)
  {
    auto pose = msg.transform;
    std::unique_lock<std::mutex> lck(mtx);
    pos_ = {pose.translation.x, pose.translation.y, pose.translation.z};
    quat_ = {pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z};
    is_updated = true;
  }

  bool is_updated;
  mutable std::mutex mtx;
  arma::vec pos_;
  arma::vec quat_;
  ros::Subscriber sub;
  ros::NodeHandle nh;
};



} // namespace ros_lib_

} // namespace as64_

#endif // AS64_ROS_LIB_TRANSFORM_READER_H

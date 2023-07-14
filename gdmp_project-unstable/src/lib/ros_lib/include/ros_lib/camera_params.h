
#ifndef AS64_ROS_LIB_CAMERA_PARAMS_H
#define AS64_ROS_LIB_CAMERA_PARAMS_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros_lib/_utils/semaphore.h>
#include <Eigen/Dense>

namespace as64_
{

namespace ros_lib_
{
  
class CameraParams
{
public:
  CameraParams() {}

  bool readFromTopic(const std::string &topic_name, unsigned timeout_ms=500)
  {
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(topic_name, 1, &CameraParams::readInfoCallback, this);
    bool success = sem.wait_for(timeout_ms);
    return success;
  }

  sensor_msgs::CameraInfo getInfo() const { return cam_info; }

  Eigen::Matrix3d getCamProjMat() const { return Eigen::Map<const Eigen::Matrix3d>(&cam_info.K[0]).transpose(); }

protected:

  void readInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    cam_info = *msg;
    sem.notify();
  }

  sensor_msgs::CameraInfo cam_info;
  ros_lib::Semaphore sem;
};

} // namespace ros_lib_

} // namespace as64_

#endif // AS64_ROS_LIB_CAMERA_PARAMS_H


#ifndef AS64_ROS_LIB_IMAGE_READER_H
#define AS64_ROS_LIB_IMAGE_READER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros_lib/_utils/semaphore.h>
#include <ros_lib/_utils/mtx_var.h>

namespace as64_
{

namespace ros_lib_
{
  
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

class ImageReader
{
public:
  ImageReader(const std::string &image_topic, const std::string &depth_topic="")
  {
    if (depth_topic.empty())
    {
      sub = nh.subscribe(image_topic, 1, &ImageReader::image_callback, this);
    }
    else
    {
      has_depth = true;
      image_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, image_topic, 1));
      depth_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_topic, 1));
      sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *depth_sub));
      sync->registerCallback(boost::bind(&ImageReader::image_depth_callback, this, _1, _2));
    }
  }

  bool readNewFrame(unsigned timeout_ms) const
  {
    img_request.set(true);
    return sem.wait_for(timeout_ms);
  }

  cv::Mat getRGB() const
  {
    return img;
  }

  cv::Mat getDepth() const
  {
    return depth;
  }

  bool hasDepth() const { return has_depth; }

private:

  cv::Mat image_msg_to_cv_mat(const sensor_msgs::ImageConstPtr& msg, const std::string &encoding)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try { cv_ptr = cv_bridge::toCvCopy(msg, encoding); }
    catch (cv_bridge::Exception& e) { throw std::runtime_error(std::string("cv_bridge exception: ") + e.what()); }
    return cv_ptr->image;
  }

  void image_depth_callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg)
  {
    if (!img_request.read()) return;

    img_request.set(false);

    img = image_msg_to_cv_mat(img_msg, sensor_msgs::image_encodings::RGB8);
    depth = image_msg_to_cv_mat(depth_msg, "16UC1");

    sem.notify();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!img_request.read()) return;

    img = image_msg_to_cv_mat(msg, sensor_msgs::image_encodings::BGR8);

    img_request.set(false);
    sem.notify();
  }

  ros::NodeHandle nh;
  ros::Subscriber sub;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

  mutable ros_lib::MtxVar<bool> img_request;
  mutable ros_lib::Semaphore sem;
  cv::Mat img;
  cv::Mat depth;

  bool has_depth=false;
};

} // namespace ros_lib_

} // namespace as64_

#endif // AS64_ROS_LIB_IMAGE_READER_H

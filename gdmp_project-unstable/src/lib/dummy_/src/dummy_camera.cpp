#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_broadcaster.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_to_pointcloud(const cv::Mat &rgb_image, const cv::Mat &depth_image, double fx, double fy, double cx, double cy)
{
  // Convert the RGB image to a PCL PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud->width = rgb_image.cols;
  pointcloud->height = rgb_image.rows;
  pointcloud->is_dense = false;
  pointcloud->points.resize(rgb_image.cols * rgb_image.rows);
  for (int i = 0; i < rgb_image.rows; ++i) {
    for (int j = 0; j < rgb_image.cols; ++j) {
      pcl::PointXYZRGB& point = pointcloud->points[i * rgb_image.cols + j];
      point.x = (j - cx) * depth_image.at<uint16_t>(i, j) / 1000.0 / fx;
      point.y = (i - cy) * depth_image.at<uint16_t>(i, j) / 1000.0 / fy;
      point.z = depth_image.at<uint16_t>(i, j) / 1000.0;
      point.r = rgb_image.at<cv::Vec3b>(i, j)[2];
      point.g = rgb_image.at<cv::Vec3b>(i, j)[1];
      point.b = rgb_image.at<cv::Vec3b>(i, j)[0];
    }
  }

  return pointcloud;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_camera");

  ros::NodeHandle nh("~");
  std::string path = ros::package::getPath("grasp_obj_controller") + "/";

  // ------ load topic names --------
  std::string robot_cam_tf_topic;
  std::string cam_info_topic;
  std::string img_topic;
  std::string depth_topic;
  std::string pointcloud_topic;
  std::string camera_frame = "dummy_camera_frame";
  std::string robot_base_frame;
  if (!nh.getParam("robot_cam_tf_topic", robot_cam_tf_topic)) throw std::runtime_error("Failed to read param \" robot_cam_tf_topic\"...\n");
  if (!nh.getParam("cam_info_topic", cam_info_topic)) throw std::runtime_error("Failed to read param \" cam_info_topic\"...\n");
  if (!nh.getParam("image_topic", img_topic)) throw std::runtime_error("Failed to read param \" image_topic\"...\n");
  if (!nh.getParam("depth_image_topic", depth_topic)) throw std::runtime_error("Failed to read param \" depth_image_topic\"...\n");
  if (!nh.getParam("pointcloud_topic", pointcloud_topic)) throw std::runtime_error("Failed to read param \" pointcloud_topic\"...\n");
  if (!nh.getParam("robot_base_frame", robot_base_frame)) throw std::runtime_error("Failed to read param \" robot_base_frame\"...\n");

  // ------ create messages --------
  geometry_msgs::TransformStamped robot_cam_tf_msg;
  std::vector<double> robot_cam_tf_values;
  if (!nh.getParam("robot_cam_tf", robot_cam_tf_values)) throw std::runtime_error("Failed to read param \" robot_cam_tf\"...\n");
  robot_cam_tf_msg.transform.translation.x = robot_cam_tf_values[0];
  robot_cam_tf_msg.transform.translation.y = robot_cam_tf_values[1];
  robot_cam_tf_msg.transform.translation.z = robot_cam_tf_values[2];
  robot_cam_tf_msg.transform.rotation.w = robot_cam_tf_values[3];
  robot_cam_tf_msg.transform.rotation.x = robot_cam_tf_values[4];
  robot_cam_tf_msg.transform.rotation.y = robot_cam_tf_values[5];
  robot_cam_tf_msg.transform.rotation.z = robot_cam_tf_values[6];

  sensor_msgs::CameraInfo cam_info_msg;
  std::vector<double> cam_info_K;
  if (!nh.getParam("cam_info_K", cam_info_K)) throw std::runtime_error("Failed to read param \" cam_info_K\"...\n");
  for (int i=0; i<9; i++) cam_info_msg.K[i] = cam_info_K[i];
  double fx = cam_info_K[0];
  double cx = cam_info_K[2];
  double fy = cam_info_K[4];
  double cy = cam_info_K[5];

  std::string img_path, depth_path;
  if (!nh.getParam("img_path", img_path)) throw std::runtime_error("Failed to read param \" img_path\"...\n");
  if (!nh.getParam("depth_path", depth_path)) throw std::runtime_error("Failed to read param \" depth_path\"...\n");
  cv::Mat img = cv::imread(path + img_path, cv::IMREAD_COLOR);
  cv::Mat depth = cv::imread(path + depth_path, cv::IMREAD_ANYDEPTH);

  cv::Mat rgb_img;
  cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb_img).toImageMsg();
  sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", depth).toImageMsg();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = image_to_pointcloud(img, depth, fx, fy, cx, cy);
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = camera_frame;

  // ------ create publishers --------
  ros::Publisher robot_cam_tf_pub = nh.advertise<geometry_msgs::TransformStamped>(robot_cam_tf_topic, 1);
  ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher img_pub = it.advertise(img_topic, 1);
  image_transport::Publisher depth_pub = it.advertise(depth_topic, 1);

  ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1);

  tf::TransformBroadcaster broadcaster;
  // Create a transform message
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(robot_cam_tf_values[0], robot_cam_tf_values[1], robot_cam_tf_values[2]));
  transform.setRotation(tf::Quaternion(robot_cam_tf_values[4], robot_cam_tf_values[5], robot_cam_tf_values[6], robot_cam_tf_values[3]));

  // ------ publish data loop --------
  while (ros::ok())
  {
    auto t = ros::Time::now();

    robot_cam_tf_msg.header.stamp = t;
    cam_info_msg.header.stamp = t;
    img_msg->header.stamp = t;
    depth_msg->header.stamp = t;
    pointcloud_msg.header.stamp = t;

    broadcaster.sendTransform(tf::StampedTransform(transform, t, robot_base_frame, camera_frame));
    
    robot_cam_tf_pub.publish(robot_cam_tf_msg);
    cam_info_pub.publish(cam_info_msg);
    img_pub.publish(img_msg);
    depth_pub.publish(depth_msg);
    pointcloud_pub.publish(pointcloud_msg);

    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
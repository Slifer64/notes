#ifndef APRILTAG_ROS_APRILTAG_LISTENER_H
#define APRILTAG_ROS_APRILTAG_LISTENER_H

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <armadillo>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/semaphore.h>

namespace apriltag_ros
{

class AprilTagListener
{
public:

  /** Tag detection struct, containing the pose of the detected tag, its id, 
   *  and the operation name to which the tag corresponds.
   *  tag_id==0 means the detection is invalid.
   */ 
  struct TagDetection
  {
    TagDetection(): tag_id(-1), op_name("") {}
    Eigen::Vector3d pos;  ///< position of the tag
    Eigen::Quaterniond quat; ///< orientation of the tag
    int tag_id;     ///< id of the tag
    std::string op_name; ///< name of the operation to which the tag corresponds
  };

  /** Constructor.
   *  Loads and registers the operations and tag ids.
   *  Subscribes to the topic where the detected tags are published.
   */ 
  AprilTagListener(const std::string &config_file, const std::string &yaml_node_name="operations_tags_map");

  /** Constructor.
   *  @param[in] tag_detections_topic: topic where the raw (in the camera frame) tag detections are published.
   *  @param[in] tag_op_map: maps the operation names and the corresponding tag ids.
   */ 
  AprilTagListener(const std::string &tag_detections_topic, const std::map<std::string, std::vector<int> > &tag_op_map);

  /** Returns all the tag detections for the requested operation.
   *  @param[in] op_name: the name of the operation.
   *  @return an std::vector of the detected tags (see @AprilTagListener::TagDetection).
   */ 
  std::vector<AprilTagListener::TagDetection> getOperationDetections(const std::string &op_name);

  /** Returns the detection of the tag with id 'tag_id'.
   *  @param[in] tag_id: the identifier of tag.
   *  @return an @AprilTagListener::TagDetection. An empty dection (with tag_id=-1) is returned if the 'tag_id' is not detected.
   */ 
  AprilTagListener::TagDetection getTagDetection(int tag_id, bool reset=false);

  /** Blocks until new tags are detected or the wait time elapses. 
   *  @param[in] wait_ms: time in ms to wait (default=0, wait infinetly until a new detection arrives)
   *  @return true if new tags where detected, false otherwise.
   */ 
  bool waitForNewDetections(unsigned wait_ms=0) const;

  /** Enable/disable publishing of the detected tags to the TF.
   *  @param[in] set: flag that enables/dissables publishing.
   */ 
  void publishDetectionsToTf(bool set) { publish_tf = set; }

  /** Sets the transform to be applied to the raw detected tags.
   *  @param[in] T: a 4x4 (or 3x4) homogenous tranform [R  p; 0 0 0 1].
   *  @param[in] parent: the name of the transform's parent link.
   *  @param[in] parent: the name of the transform's child link.
   */ 
  void setTagsTrasform(const arma::mat &Tf, const std::string &parent="N/A", const std::string &child="N/A");

  /** Launches a subscriber to the topic \a tag_transform_topic, where the transform to be applied to the raw tag detections is published.
   *  @param[in] tag_transform_topic: the name of the topic where the transform to be applied to the raw tag detections is published.
   */
  void listenToTagTransformTopic(const std::string &tag_transform_topic);

  /** Stops listening to the topic \a tag_transform_topic, where the transform to be applied to the raw tag detections is published.
   */
  void stopListeningToTagTransformTopic();

  std::vector<std::string> getOperationNames() const { return op_names; }

private:

  /** Listens to the detected tags topic. Each time detections occur, clears all past detected tags and assigns the new detections to the corresponding operation poses.
   *  @param[in] msg: a message with the detected tags in an array.
   */ 
  void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg);

  // ================================================================

  std::vector<std::string> op_names;

  class TagDetectionMtx
  {
    public:
      explicit TagDetectionMtx() { offset.setIdentity(); mtx_ = new std::mutex; }

      ~TagDetectionMtx() { delete mtx_; }

      TagDetectionMtx(const TagDetectionMtx &obj)
      {
        *this = obj;
      }

      const TagDetectionMtx &operator=(const TagDetectionMtx &obj)
      {
        mtx_ = new std::mutex;
        detection_ = obj.detection_;
        offset = obj.offset;
        return *this;
      }

      AprilTagListener::TagDetection get() const
      {
        std::unique_lock<std::mutex> lck(*mtx_);
        return detection_;
      }

      void set(const AprilTagListener::TagDetection &detection)
      {
        std::unique_lock<std::mutex> lck(*mtx_);
        detection_ = detection;

        Eigen::Vector3d p1 = detection_.pos;

        Eigen::Isometry3d A;
        A.translation() = detection_.pos;
        A.linear() = detection_.quat.toRotationMatrix();
        A = A*offset;
        detection_.pos = A.translation();
        detection_.quat = Eigen::Quaterniond( A.linear() );
      }

      std::string getOperation() const { return detection_.op_name; }

      void reset()
      {
        std::unique_lock<std::mutex> lck(*mtx_);
        detection_.tag_id = -1;
      }

      void setOffset(const Eigen::Vector3d &xyz, const Eigen::Vector3d &rpy)
      {
        offset.translation() = xyz;
        Eigen::Quaterniond quat = Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ());
        offset.linear() = quat.toRotationMatrix();
      }

    private:
      AprilTagListener::TagDetection detection_;
      Eigen::Isometry3d offset;
      mutable std::mutex *mtx_ = NULL;
  };

  std::map<int, TagDetectionMtx> tag_detections_map;
  std::map<std::string, std::vector<int> > op_tags_map;

  ros::Subscriber tag_sub; ///< listes to the topic where the raw (in the camera frame) detected tags are published

  ros::AsyncSpinner ros_async_spinner;

  mutable apriltag_ros::Semaphore detect_sem; ///< semaphore that signals the arrivals of new detections

  tf::StampedTransform tag_transform; ///< transform applied to each detection
  std::mutex tag_tf_mtx;

  bool publish_tf;  ///< flag for publishing the detected tags to TF

  ros::Subscriber tf_sub; ///< listens to a topic where the tranform to be applied on the detected tags is published
  void tagTransformCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);


  static void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out=std::cerr);

}; // class AprilTagListener


} // apriltag_ros

#endif // APRILTAG_ROS_APRILTAG_LISTENER_H

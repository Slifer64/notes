#include <apriltag_ros/apriltag_listener.h>

#include <XmlRpcValue.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <exception>

#include <yaml-cpp/yaml.h>

namespace apriltag_ros
{

#define AprilTagListener_fun_ std::string("[AprilTagListener::") + __func__ + "]: "

AprilTagListener::AprilTagListener(const std::string &config_file, const std::string &yaml_node_name) : 
  ros_async_spinner(1), 
  tag_transform(tf::Transform::getIdentity(), ros::Time::now(), "N/A", "N/A")
{
  ros::NodeHandle nh("~");

  struct Offset
  {
    Eigen::Vector3d xyz;
    Eigen::Vector3d rpy;
  };

  std::map<int, Offset> id_offset_map;

  // ==========  Parse operations - tags  ============

  YAML::Node config = YAML::LoadFile(config_file);

  // ------------  parse ${yaml_node_name}  --------------
  YAML::Node op_tags_node;
  if ( !YAML::getParam(config, yaml_node_name, op_tags_node) )
    throw std::runtime_error(AprilTagListener_fun_+"Failed to load param '" + yaml_node_name + "'...");
  if ( !op_tags_node.IsSequence() )
    throw std::runtime_error(AprilTagListener_fun_+"'" + yaml_node_name + "' must be an array...");

  for (int i=0; i<op_tags_node.size(); i++)
  {
    std::string i_str = std::to_string(i);
    const YAML::Node &node_i = op_tags_node[i];
    if ( !node_i.IsMap() )
      throw std::runtime_error(AprilTagListener_fun_+"'" + yaml_node_name + "[" + i_str + "]' must be a struct...");

    std::vector<int> tag_ids;

    std::string name;
    if ( !YAML::getParam(node_i, "name", name) )
      throw std::runtime_error(AprilTagListener_fun_+"Failed to load param '" + yaml_node_name + "[" + i_str + "].name'...");

    YAML::Node tags_node;
    if ( !YAML::getParam(node_i, "tags", tags_node) )
      throw std::runtime_error(AprilTagListener_fun_+"Failed to load param '" + yaml_node_name + "[" + i_str + "].tags'...");

    if ( !tags_node.IsSequence() )
      throw std::runtime_error(AprilTagListener_fun_+ yaml_node_name + "[" + i_str + "].tags' must be an array...");

    for (int j=0; j<tags_node.size(); j++)
    {
      std::string j_str = std::to_string(j);
      const YAML::Node &tag_j_node = tags_node[j];
      if ( !tag_j_node.IsMap() )
        throw std::runtime_error(AprilTagListener_fun_+"'" + yaml_node_name + "[" + i_str + "].tags[" + j_str + "]' must be a struct...");

      int id;
      if ( !YAML::getParam(tag_j_node, "id", id) )
        throw std::runtime_error(AprilTagListener_fun_+"Failed to load param '" + yaml_node_name + "[" + i_str + "].tags[" + j_str + "].id'...");

      tag_ids.push_back(id);

      YAML::Node offset_node;
      if ( !YAML::getParam(tag_j_node, "offset", offset_node) )
        throw std::runtime_error(AprilTagListener_fun_+"Failed to load param '" + yaml_node_name + "[" + i_str + "].tags[" + j_str + "].offset'...");

      if ( !offset_node.IsMap() )
        throw std::runtime_error(AprilTagListener_fun_+ yaml_node_name + "[" + i_str + "].tags[" + j_str + "].offset' must be a struct...");

      Offset offset;
      if ( !YAML::getParam(offset_node, "xyz", offset.xyz) )
        throw std::runtime_error(AprilTagListener_fun_+"Failed to load param '" + yaml_node_name + "[" + i_str + "].tags[" + j_str + "].offset.xyz'...");
      if ( !YAML::getParam(offset_node, "rpy", offset.rpy) )
        throw std::runtime_error(AprilTagListener_fun_+"Failed to load param '" + yaml_node_name + "[" + i_str + "].tags[" + j_str + "].offset.rpy'...");

      id_offset_map[id] = offset;
    }

    op_tags_map[name] = tag_ids;
  }

  for (auto it=op_tags_map.begin(); it != op_tags_map.end(); it++)
  {
    TagDetection tag_;
    op_names.push_back(it->first);
    tag_.op_name = it->first;
    for (int id : it->second)
    {
      tag_.tag_id = -1; // to signal that initially the tag is not detected
      TagDetectionMtx tag_mtx;
      tag_mtx.set(tag_);

      Offset offset = id_offset_map.find(id)->second;
      tag_mtx.setOffset(offset.xyz, offset.rpy);
      tag_detections_map[id] = tag_mtx;
    }
  }
  // --------------------------

  std::string tag_detections_topic;
  if ( !YAML::getParam(config, "tag_detections_topic", tag_detections_topic) )
    throw std::runtime_error(AprilTagListener_fun_+"Failed to load param 'tag_detections_topic'...");

  bool publish_detections_to_tf = false;
  YAML::getParam(config, "publish_detections_to_tf", publish_detections_to_tf);
  publishDetectionsToTf(publish_detections_to_tf);

  // ==========  subscribe to the topic where the detected tags are published  ============
  ros::NodeHandle node; // public node
  tag_sub = node.subscribe(tag_detections_topic, 1, &AprilTagListener::tagDetectionsCallback, this);

  //tf_sub = node.subscribe(tag_transform_topic, 1, &AprilTagListener::tagTransformCallback, this);

  ros_async_spinner.start();
  
}

AprilTagListener::AprilTagListener(const std::string &tag_detections_topic, const std::map<std::string, std::vector<int> > &op_tags_map) : 
  ros_async_spinner(1), 
  tag_transform(tf::Transform::getIdentity(), ros::Time::now(), "N/A", "N/A")
{
  this->op_tags_map = op_tags_map;

  for (auto it=op_tags_map.begin(); it != op_tags_map.end(); it++)
  {
    TagDetection tag_;
    op_names.push_back(it->first);
    tag_.op_name = it->first;
    for (int id : it->second)
    {
      tag_.tag_id = -1; // to signal that initially the tag is not detected
      TagDetectionMtx tag_mtx;
      tag_mtx.set(tag_);
      tag_detections_map[id] = tag_mtx;
    }
  }

  tag_sub = ros::NodeHandle().subscribe(tag_detections_topic, 1, &AprilTagListener::tagDetectionsCallback, this);
  ros_async_spinner.start();
}

std::vector<AprilTagListener::TagDetection> AprilTagListener::getOperationDetections(const std::string &op_name) 
{ 
  std::vector<AprilTagListener::TagDetection> detections;

  auto it = op_tags_map.find(op_name);
  if (it == op_tags_map.end()) throw std::runtime_error("[AprilTagListener::getOperationPoses]: Invalid operation '" + op_name + "'...");

  const std::vector<int> &tag_ids = it->second;
  for (int id : tag_ids)
  {
    AprilTagListener::TagDetection detection = getTagDetection(id);
    if (detection.tag_id >= 0) detections.push_back(detection);
  }

  return detections; 
}

AprilTagListener::TagDetection AprilTagListener::getTagDetection(int tag_id, bool reset)
{
  auto it = tag_detections_map.find(tag_id);
  if (it == tag_detections_map.end()) return AprilTagListener::TagDetection(); // the tag doesn't correspond to any operation

  auto tag = it->second.get();
  if (reset) it->second.reset();
  return tag;
}

bool AprilTagListener::waitForNewDetections(unsigned wait_ms) const
{
  // ros:spinOnce();

  if (!wait_ms) 
  {
    detect_sem.wait();
    return true;
  }

  return detect_sem.wait_for(wait_ms);
}

void AprilTagListener::setTagsTrasform(const arma::mat &Tf, const std::string &parent, const std::string &child)
{
  std::unique_lock<std::mutex> lck(tag_tf_mtx);

  tag_transform.setOrigin( tf::Vector3( Tf(0,3), Tf(1,3), Tf(2,3) ) );
  tag_transform.setBasis( tf::Matrix3x3( Tf(0,0), Tf(0,1), Tf(0,2), Tf(1,0), Tf(1,1), Tf(1,2), Tf(2,0), Tf(2,1), Tf(2,2) ) );
  tag_transform.frame_id_ = parent;
  tag_transform.child_frame_id_ = child;
  tag_transform.stamp_ = ros::Time::now();
}

void AprilTagListener::listenToTagTransformTopic(const std::string &tag_transform_topic)
{
  tf_sub = ros::NodeHandle().subscribe(tag_transform_topic, 1, &AprilTagListener::tagTransformCallback, this);
}

void AprilTagListener::stopListeningToTagTransformTopic() 
{ 
  tag_sub.shutdown(); 
}

void AprilTagListener::tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg)
{
  static tf::TransformBroadcaster tf_br;
  // static tf::TransformListener tf_listen;

  // clear previous detections
  for (auto it=tag_detections_map.begin(); it!=tag_detections_map.end(); it++) it->second.reset();

  int n = msg->detections.size();

  // std::vector<apriltag_ros::AprilTagDetection> tags(msg->detections.size());
  // std::copy( msg->detections.begin(), msg->detections.end(), tags.begin() );

  tf::StampedTransform Tf_base_camera;
  {
    std::unique_lock<std::mutex> lck(tag_tf_mtx);
    Tf_base_camera = tag_transform;
  }


  // try
  // {
  //   tf_listen.lookupTransform("ur5_base_link", "lwr4pCam_xtion_pro_rgb_optical_frame", ros::Time(0), Tf_base_camera);
  // }
  // catch (tf::TransformException ex)
  // {
  //   ROS_ERROR("%s",ex.what());
  //   ros::Duration(1.0).sleep();
  // }
    
  // Process each tag detection and assign it to the corresponding opetation.
  auto it = msg->detections.begin();
  for (int i=0; i<n; i++)
  {
    const apriltag_ros::AprilTagDetection &tag = *it++;
    int tag_id = tag.id[0];
    
    // find the operation id for the detected tag
    auto tag_it = tag_detections_map.find(tag_id);
    if (tag_it == tag_detections_map.end()) continue; // print warning msg?

    TagDetectionMtx &tag_mtx = tag_it->second;

    std::string op_name = tag_mtx.getOperation();

    TagDetection td;
    auto pose = tag.pose.pose.pose;

    // check tf frame ids
    if ( tag.pose.header.frame_id.compare(Tf_base_camera.child_frame_id_) )
    {
      PRINT_WARNING_MSG(AprilTagListener_fun_ + 
      "tag base frame is different from tag transform child frame:\n" \
      "- tag base frame           : " + tag.pose.header.frame_id + "\n" + \
      "- tag transform child frame: " + Tf_base_camera.child_frame_id_ + "\n");
    }

    tf::Vector3 pos(pose.position.x, pose.position.y, pose.position.z);
    pos = Tf_base_camera * pos;
    tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    quat = Tf_base_camera * quat;

    td.pos = {pos.x(), pos.y(), pos.z()};
    td.quat = {quat.w(), quat.x(), quat.y(), quat.z()};
    td.tag_id = tag_id;
    td.op_name = op_name; 

    tag_mtx.set(td);

    if (publish_tf)
    {
      tf::StampedTransform tag_tf;
      tag_tf.setOrigin( pos );
      tag_tf.setRotation( quat );
      tag_tf.stamp_ = ros::Time::now();
      tag_tf.frame_id_ = Tf_base_camera.frame_id_;
      tag_tf.child_frame_id_ = op_name + "_tag_" + std::to_string(tag_id);
      tf_br.sendTransform(tag_tf);
    }
  }

  detect_sem.notify();
}

void AprilTagListener::tagTransformCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  std::unique_lock<std::mutex> lck(tag_tf_mtx);

  tf::transformStampedMsgToTF(*msg, tag_transform);

  // geometry_msgs::Point pos = msg->position;
  // geometry_msgs::Quaternion quat = msg->orientation;

  // tag_transform.setOrigin( tf::Vector3(pos.x, pos.y, pos.z) );
  // tag_transform.setRotation( tf::Quaternion(quat.x, quat.y, quat.z, quat.w) );
  
  // tf::transformMsgToTF( const geometry_msgs::Transform &msg, Transform &bt) 

  // std::cerr << AprilTagListener_fun_ + "Got transform!\n";	
}

void AprilTagListener::PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

} // apriltag_ros


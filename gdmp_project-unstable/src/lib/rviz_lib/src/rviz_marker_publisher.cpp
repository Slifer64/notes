#include <rviz_lib/rviz_marker_publisher.h>

namespace as64_
{

namespace rviz_
{

#define RvizMarkerPublisher_fun_ std::string("[RvizMarkerPublisher::") + __func__ + "]: "

const std::string LOGNAME = "rviz_";

// ================================================
// ===============  CLASS Color  ==================
// ================================================

Color Color::BLACK = Color(0, 0, 0);
Color Color::BROWN = Color(0.597, 0.296, 0);
Color Color::BLUE = Color(0.1, 0.1, 0.8);
Color Color::CYAN = Color(0, 1.0, 1.0);
Color Color::GREY = Color(0.9, 0.9, 0.9);
Color Color::DARK_GREY = Color(0.6, 0.6, 0.6);
Color Color::GREEN = Color(0.1, 0.8, 0.1);
Color Color::LIME_GREEN = Color(0.6, 1.0, 0.2);
Color Color::MAGENTA = Color(1.0, 0, 1.0);
Color Color::ORANGE = Color(1.0, 0.5, 0.0);
Color Color::PURPLE = Color(0.597, 0, 0.597);
Color Color::RED = Color(0.8, 0.1, 0.1);
Color Color::PINK = Color(1.0, 0.4, 1.0);
Color Color::WHITE = Color(1.0, 1.0, 1.0);
Color Color::YELLOW = Color(1.0, 1.0, 0);
Color Color::TRANSLUCENT = Color(0.1, 0.1, 0.1, 0.25);
Color Color::TRANSLUCENT_LIGHT = Color(0.1, 0.1, 0.1, 0.1);
Color Color::TRANSLUCENT_DARK = Color(0.1, 0.1, 0.1, 0.5);

Color::Color(double r, double g, double b, double alpha)
{
  setColorParam(r, &r_);
  setColorParam(g, &g_);
  setColorParam(b, &b_);
  setColorParam(alpha, &alpha_);
}

void Color::setColorParam(double c_ref, double *c)
{
  if (c_ref < 0)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Color parameter is < 0: " << c_ref << ". Setting to 0.");
    c_ref = 0;
  }
  if (c_ref > 1)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Color parameter is > 1: " << c_ref << ". Setting to 1.");
    c_ref = 1;
  }
  *c = c_ref;
}


// ================================================
// ===============  CLASS Scale  ==================
// ================================================


Scale::Scale(double scale)
{
  if (scale < 0)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Scale is negative: " << scale << ". Setting to 0.");
    scale = 0;
  }
  scale_ = scale;
}

Scale Scale::XXXXSMALL = Scale(2*0.001);
Scale Scale::XXXSMALL = Scale(2*0.0025);
Scale Scale::XXSMALL = Scale(2*0.005);
Scale Scale::XSMALL = Scale(2*0.0065);
Scale Scale::SMALL = Scale(2*0.0075);
Scale Scale::MEDIUM = Scale(2*0.01);
Scale Scale::LARGE = Scale(2*0.025);
Scale Scale::XLARGE = Scale(2*0.05);
Scale Scale::XXLARGE = Scale(2*0.075);
Scale Scale::XXXLARGE = Scale(2*0.1);
Scale Scale::XXXXLARGE = Scale(2*0.5);


// ==============================================================
// ===============  CLASS RvizMarkerPublisher  ==================
// ==============================================================


RvizMarkerPublisher::RvizMarkerPublisher(const std::string &marker_array_topic, const std::string &base_frame)
{
  setBaseFrame(base_frame);

  ros::NodeHandle nh;

  this->marker_array_topic_ = marker_array_topic;

  rviz_pub = nh.advertise<visualization_msgs::MarkerArray>( marker_array_topic, 10, false);

  waitForSubscriber(0.5, false);
}

bool RvizMarkerPublisher::waitForSubscriber(double wait_time, bool blocking)
{
  // wait at most this amount of time
  ros::Time max_time(ros::Time::now() + ros::Duration(wait_time));

  // How often to check for subscribers
  ros::Rate poll_rate(200);

  if (blocking && rviz_pub.getNumSubscribers() == 0)
  {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Topic '" << rviz_pub.getTopic() << "' waiting for subscriber...");
  }

  // Wait for subscriber
  while (!rviz_pub.getNumSubscribers())
  {
    if (!blocking && ros::Time::now() > max_time)  // Check if timed out
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "No subscribers connected to topic '" << rviz_pub.getTopic() << "' within "
                                      << wait_time << " sec. Initial published visual messages may be lost.");
      return false;
    }
    ros::spinOnce();

    // Sleep
    poll_rate.sleep();

    if (!ros::ok()) return false;
  }

  return true;
}

void RvizMarkerPublisher::setBaseFrame(const std::string &base_frame)
{
  base_frame_ = base_frame;
}

void RvizMarkerPublisher::setGlobalScale(double gb_scale)
{
  if (gb_scale < 0)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Global scale is negative: " << gb_scale << ". Setting to 0.");
    gb_scale = 0;
  }
  global_scale_ = gb_scale;
}

double RvizMarkerPublisher::getGlobalScale() const
{
  return global_scale_;
}

void RvizMarkerPublisher::setFrameLocking(bool set)
{
  frame_locked_ = set;
}

RvizMarkerPublisher::MarkersNs *RvizMarkerPublisher::getMarkersNamespaceStruct(const std::string &ns)
{
  MarkersNs *s;

  std::map<std::string, MarkersNs>::iterator it = ns_map.find(ns);
  if (it != ns_map.end()) s = &(it->second);
  else s = &(ns_map[ns] = MarkersNs(ns));
  
  return s;
}

Eigen::Quaterniond RvizMarkerPublisher::directionToOrient(const Eigen::Vector3d &direction, const Eigen::Vector3d &ref_direction)
{
  return Eigen::Quaterniond().setFromTwoVectors(ref_direction, direction);
}
  
// =========================================================

// -------------- Oriented Path ---------------

// -------------- Path ---------------

void RvizMarkerPublisher::publishPath(const std::vector<Eigen::Vector3d> &path, rviz_::Color color, rviz_::Scale line_width, const std::string &ns)
{
  int path_size = path.size();
  if (path_size < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Skipping path because " << path_size << " points passed in.");
    return;
  }

  MarkersNs *s = getMarkersNamespaceStruct(ns);

  // Create the cylinders
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(path_size-1);
  int k = 0;
  for (size_t i = 1; i < path_size; i++)
  {
    Eigen::Vector3d p1(path[i-1](0), path[i-1](1), path[i-1](2));
    Eigen::Vector3d p2(path[i](0), path[i](1), path[i](2));
    if ( (p2-p1).norm() == 0 ) continue;
    double radius = line_width() / 2;

    markers.markers[k++] = createCylinderMarker(p1, p2, radius, color, s);
  }
  markers.markers.resize(k);
  publishMarkers(markers);
}

void RvizMarkerPublisher::publishPath(const arma::mat &path, rviz_::Color color, rviz_::Scale line_width, const std::string &ns)
{
  int path_size = path.n_cols;
  if (path_size < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Skipping path because " << path_size << " points passed in.");
    return;
  }

  MarkersNs *s = getMarkersNamespaceStruct(ns);

  // Create the cylinders
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(path_size-1);
  int k = 0;
  for (size_t i = 1; i < path_size; i++)
  {
    Eigen::Vector3d p1(path(0,i-1), path(1,i-1), path(2,i-1));
    Eigen::Vector3d p2(path(0,i), path(1,i), path(2,i));
    if ( (p2-p1).norm() == 0 ) continue;
    double radius = line_width() / 2;

    markers.markers[k++] = createCylinderMarker(p1, p2, radius, color, s);
  }
  markers.markers.resize(k);
  publishMarkers(markers);
}

// -------------- Oriented Path ---------------

void RvizMarkerPublisher::publishOrientedPath(const arma::mat &path, const arma::mat &orient, unsigned n_frames, rviz_::Color color, rviz_::Scale line_width, rviz_::Scale orient_frame_scale, const std::string &ns)
{
  if (path.n_cols != orient.n_cols)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "pos and orient data must have the same size.\n");
    return;
  }

  std::vector<Eigen::Vector3d> path2(path.n_cols);
  std::vector<Eigen::Quaterniond> orient2(path.n_cols);
  for (int j=0; j<path2.size(); j++)
  {
    path2[j] = Eigen::Vector3d( path(0,j), path(1,j), path(2,j) );
    orient2[j] = Eigen::Quaterniond( orient(0,j), orient(1,j), orient(2,j), orient(3,j) );
  }
  publishOrientedPath(path2, orient2, n_frames, color, line_width, orient_frame_scale, ns);
}

void RvizMarkerPublisher::publishOrientedPath(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Quaterniond> &orient, unsigned n_frames, rviz_::Color color, rviz_::Scale line_width, rviz_::Scale orient_frame_scale, const std::string &ns)
{
  if (path.size() != orient.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "pos and orient data must have the same size.\n");
    return;
  }

  if (n_frames < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "At least two frame are required. Setting n_frames=2\n");
    n_frames = 2;
  }
  publishOrientedPath(path, orient, getEquidistantPathPointIndices(path,n_frames), color, line_width, orient_frame_scale, ns);
}

void RvizMarkerPublisher::publishOrientedPath(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Quaterniond> &orient, const std::vector<unsigned> orient_ind, rviz_::Color color, rviz_::Scale line_width, rviz_::Scale orient_frame_scale, const std::string &ns)
{
  if (path.size() != orient.size())
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "pos and orient data must have the same size.\n");
    return;
  }
  
  MarkersNs *s = getMarkersNamespaceStruct(ns);

  publishPath(path, color, line_width, ns);
  std::vector<rviz_::Color> axes_color;
  axes_color.push_back(x_axis_color);
  axes_color.push_back(y_axis_color);
  axes_color.push_back(z_axis_color);
  visualization_msgs::MarkerArray markers;
  for (int i=0; i<orient_ind.size(); i++)
  {
    unsigned k = orient_ind[i];
    visualization_msgs::MarkerArray markers_i = createFrameMarker(path[k], orient[k], axis_width_, axis_length_,  axes_color, s);
    appendMarkers(&markers, markers_i);
  }
  publishMarkers(markers);
}

std::vector<unsigned> RvizMarkerPublisher::getEquidistantPathPointIndices(const std::vector<Eigen::Vector3d> &path, int n_ind)
{
  double total_len = 0;

  int n_data = path.size();
  for (int j=0; j<n_data-1; j++) total_len += (path[j+1] - path[j]).lpNorm<2>();

  double i_len = total_len / (n_ind-1);

  std::vector<unsigned> ind = {0};

  double len = 0;
  for (int j=0; j<n_data-1; j++)
  {
    len += (path[j+1] - path[j]).lpNorm<2>();
    if (len > i_len)
    {
      ind.push_back(j);
      len = 0;
    }
  }

  ind.push_back(n_data-1);

  return ind;
}

// -------------- Frame ---------------

void RvizMarkerPublisher::publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, const std::string &ns)
{
  publishFrame(pos, orient, axis_width_, axis_length_, ns);
}

void RvizMarkerPublisher::publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale line_width, rviz_::Scale axis_length, const std::string &ns)
{
  std::vector<rviz_::Color> axes_color;
  axes_color.push_back(x_axis_color);
  axes_color.push_back(y_axis_color);
  axes_color.push_back(z_axis_color);
  publishFrame(pos, orient, line_width, axis_length, axes_color, ns);
}

void RvizMarkerPublisher::publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale line_width, rviz_::Scale axis_length,  const std::vector<rviz_::Color> &axes_color, const std::string &ns)
{
  if (axes_color.size() != 3)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "The axes colors vector is " << axes_color.size() << " != 3\n");
    return;
  }

  publishMarkers( createFrameMarker(pos, orient, line_width, axis_length,  axes_color, getMarkersNamespaceStruct(ns) ) );
}

void RvizMarkerPublisher::publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale scale, const std::vector<rviz_::Color> &axes_color, const std::string &ns)
{
  publishFrame(pos, orient, scale()*axis_width_, scale()*axis_length_, axes_color, ns);
}

void RvizMarkerPublisher::publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale scale, const std::string &ns)
{
  publishFrame(pos, orient, scale()*axis_width_, scale()*axis_length_, ns);
}

visualization_msgs::MarkerArray RvizMarkerPublisher::createFrameMarker(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale line_width, rviz_::Scale axis_length,  const std::vector<rviz_::Color> &axes_color, MarkersNs *s)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(3);
  markers.markers[0] = createCylinderMarker( pos, pos + orient*Eigen::Vector3d(axis_length(),0,0), line_width()/2, axes_color[0], s);
  markers.markers[1] = createCylinderMarker( pos, pos + orient*Eigen::Vector3d(0,axis_length(),0), line_width()/2, axes_color[1], s);
  markers.markers[2] = createCylinderMarker( pos, pos + orient*Eigen::Vector3d(0,0,axis_length()), line_width()/2, axes_color[2], s);

  return markers;
}

// -------------- Capsule ---------------

void RvizMarkerPublisher::publishCapsule(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  publishMarkers( createCapsuleMarker(center, orient, height, radius, color, getMarkersNamespaceStruct(ns)) );
}

void RvizMarkerPublisher::publishCapsule(const Eigen::Vector3d &center, const Eigen::Vector3d &z_ax, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  publishCapsule(center, directionToOrient(z_ax), height, radius, color, ns);
}

void RvizMarkerPublisher::publishCapsule(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  publishCapsule( (pos1 + pos2)/2 , directionToOrient(pos2-pos1), (pos2-pos1).lpNorm<2>(), radius, color, ns);
}

visualization_msgs::MarkerArray RvizMarkerPublisher::createCapsuleMarker(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, MarkersNs *s)
{
  Eigen::Vector3d v = orient * Eigen::Vector3d(0,0,height()/2);

  visualization_msgs::MarkerArray markers;
  markers.markers.resize(3);
  markers.markers[0] = createSphere( center-v , orient, radius, color, s);
  markers.markers[1] = createSphere( center+v , orient, radius, color, s);
  markers.markers[2] = createCylinderMarker(center, orient, height, radius, color, s);
  
  return markers;
}

// -------------- Cylinder ---------------

void RvizMarkerPublisher::publishCylinder(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  publishMarker( createCylinderMarker(center, orient, height, radius, color, getMarkersNamespaceStruct(ns) ) );
}

void RvizMarkerPublisher::publishCylinder(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  publishMarker( createCylinderMarker(pos1, pos2, radius, color, getMarkersNamespaceStruct(ns) ) );
}

void RvizMarkerPublisher::publishCylinder(const Eigen::Vector3d &center, const Eigen::Vector3d &z_ax, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  publishCylinder(center, directionToOrient(z_ax), height, radius, color, ns);
}

visualization_msgs::Marker RvizMarkerPublisher::createCylinderMarker(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, rviz_::Scale radius, rviz_::Color color, MarkersNs *s)
{ 
  Eigen::Vector3d center = (pos2 + pos1) / 2;
  Eigen::Vector3d p12 = pos2 - pos1;
  double height = p12.lpNorm<2>();

  // find the quaternion that transform the cylinders nominal axis (z-axis) to the new axis (along the line of pos1, pos2)
  Eigen::Quaterniond orient = directionToOrient(p12); // WARNING: if p12=0, returns nan...

  return createCylinderMarker(center, orient, height, radius, color, s);
}

visualization_msgs::Marker RvizMarkerPublisher::createCylinderMarker(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, MarkersNs *s)
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = base_frame_;
  marker.ns = s->ns;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = s->id++;
  marker.lifetime = ros::Duration(lifetime_);

  // std::cerr << "[createCylinderMarker]: ns:" << ns << ", id:" << id << ", action=" << marker.action << "\n";

  Eigen::Quaterniond quat = orient.normalized();

  // Set the pose
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.pose.orientation.w = quat.w();
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  
  // Set marker size
  marker.scale.x = radius();
  marker.scale.y = radius();
  marker.scale.z = height();

  // Set marker color
  marker.color.r = color.r_;
  marker.color.g = color.g_;
  marker.color.b = color.b_;
  marker.color.a = color.alpha_;

  marker.frame_locked = frame_locked_;

  s->markers.markers.push_back(marker);

  return marker;
}

// -------------- Sphere ---------------

void RvizMarkerPublisher::publishSphere(const Eigen::Vector3d &center, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{ 
  publishSphere(center, Eigen::Vector3d(0,0,1), radius, color, ns);
}

void RvizMarkerPublisher::publishSphere(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  visualization_msgs::Marker marker = createSphere(center, orient, radius, color, getMarkersNamespaceStruct(ns));
  publishMarker(marker);
}

void RvizMarkerPublisher::publishSphere(const Eigen::Vector3d &center, const Eigen::Vector3d &z_ax, rviz_::Scale radius, rviz_::Color color, const std::string &ns)
{
  publishSphere(center, directionToOrient(z_ax), radius, color, ns);
}

visualization_msgs::Marker RvizMarkerPublisher::createSphere(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale radius, rviz_::Color color, MarkersNs *s)
{
  return createEllipsoid(center, orient, std::array<rviz_::Scale, 3>({radius, radius, radius}), color, s);
}

// -------------- Ellipsoid ---------------
void RvizMarkerPublisher::publishEllipsoid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, std::array<rviz_::Scale, 3> axes_len, rviz_::Color color, const std::string &ns)
{
  publishMarker(createEllipsoid(center, orient, axes_len, color, getMarkersNamespaceStruct(ns)));
}

void RvizMarkerPublisher::publishEllipsoid(const Eigen::Vector3d &center, const Eigen::Matrix3d &Sigma, rviz_::Color color, const std::string &ns)
{
  publishMarker(createEllipsoid(center, Sigma, color, getMarkersNamespaceStruct(ns)));
}

visualization_msgs::Marker RvizMarkerPublisher::createEllipsoid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, std::array<rviz_::Scale, 3> axes_len, rviz_::Color color, MarkersNs *s)
{
  // by default, the principal axis is the y-axis, so change it to be the z-axis
  static Eigen::Quaterniond orient0( Eigen::AngleAxisd(-arma::datum::pi/2, Eigen::Vector3d(1,0,0) ) );

  Eigen::Quaterniond orient2 = orient; // * orient0;

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = base_frame_;
  marker.ns = s->ns;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = s->id++;
  marker.lifetime = ros::Duration(lifetime_);

  // Set the pose
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.pose.orientation.w = orient2.w();
  marker.pose.orientation.x = orient2.x();
  marker.pose.orientation.y = orient2.y();
  marker.pose.orientation.z = orient2.z();

  // Set marker size
  marker.scale.x = 2*axes_len[0]();
  marker.scale.y = 2*axes_len[1]();
  marker.scale.z = 2*axes_len[2]();

  // Set marker color
  marker.color.r = color.r_;
  marker.color.g = color.g_;
  marker.color.b = color.b_;
  marker.color.a = color.alpha_;

  marker.frame_locked = frame_locked_;

  s->markers.markers.push_back(marker);

  return marker;
}

visualization_msgs::Marker RvizMarkerPublisher::createEllipsoid(const Eigen::Vector3d &center, const Eigen::Matrix3d &Sigma, rviz_::Color color, MarkersNs *s)
{
  auto svd = Sigma.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d ax_len = svd.singularValues().cwiseSqrt();
  Eigen::Matrix3d R = svd.matrixU();

  return createEllipsoid(center, Eigen::Quaterniond(R), {ax_len(0), ax_len(1), ax_len(2)}, color, s);
}

// -------------- Cuboid ---------------

void RvizMarkerPublisher::publishCuboid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale x_len, rviz_::Scale y_len, rviz_::Scale z_len, rviz_::Color color, const std::string &ns)
{
  publishMarker( createCuboid(center, orient, x_len, y_len, z_len, color, getMarkersNamespaceStruct(ns)) );
}

visualization_msgs::Marker RvizMarkerPublisher::createCuboid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale x_len, rviz_::Scale y_len, rviz_::Scale z_len, rviz_::Color color, MarkersNs *s)
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = base_frame_;
  marker.ns = s->ns;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = s->id++;
  marker.lifetime = ros::Duration(lifetime_);

  // Set the pose
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.pose.orientation.w = orient.w();
  marker.pose.orientation.x = orient.x();
  marker.pose.orientation.y = orient.y();
  marker.pose.orientation.z = orient.z();

  // Set marker size
  marker.scale.x = x_len();
  marker.scale.y = y_len();
  marker.scale.z = z_len();

  // Set marker color
  marker.color.r = color.r_;
  marker.color.g = color.g_;
  marker.color.b = color.b_;
  marker.color.a = color.alpha_;

  marker.frame_locked = frame_locked_;

  s->markers.markers.push_back(marker);

  return marker;
}

// -------------- Plane ---------------

void RvizMarkerPublisher::publishPlane(const Eigen::Vector3d &center, const Eigen::Vector3d &normal, rviz_::Scale width, rviz_::Scale height, rviz_::Color color, const std::string &ns)
{
  publishCuboid(center, directionToOrient(normal), width, height, 0.001, color, ns);
}

// =========================================================

// #######################################
// ############  protected  ##############
// #######################################

// =========================================================

void RvizMarkerPublisher::publishMarker(const visualization_msgs::Marker &marker)
{
  markers_to_pub.markers.push_back(marker);
  // if (auto_draw_) drawnow();
}

void RvizMarkerPublisher::publishMarkers(const visualization_msgs::MarkerArray &markers)
{
  appendMarkers(&markers_to_pub, markers);
  // if (auto_draw_) drawnow();
}

void RvizMarkerPublisher::transformMarkers(const Eigen::Isometry3d &rel_tf, const std::string &ns)
{
  std::map<std::string, MarkersNs>::iterator it = ns_map.find(ns);
  if (it == ns_map.end())
  {
    if (enable_topic_not_exist_warnings) ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Marker namespace " << ns << " does not exist.");
    return;
  }
  
  MarkersNs *s = &(it->second);
  
  for (visualization_msgs::Marker &marker : s->markers.markers)
  {
    marker.action = visualization_msgs::Marker::MODIFY;

    Eigen::Vector3d pos(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    Eigen::Quaterniond orient(marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z);

    Eigen::Isometry3d tf;
    tf.linear() = orient.toRotationMatrix();
    tf.translation() = pos;

    tf = rel_tf * tf.inverse();

    pos = tf.translation();
    orient = tf.rotation();

    marker.pose.position.x = pos(0);
    marker.pose.position.y = pos(1);
    marker.pose.position.z = pos(2);
    marker.pose.orientation.w = orient.w();
    marker.pose.orientation.x = orient.x();
    marker.pose.orientation.y = orient.y();
    marker.pose.orientation.z = orient.z();
  }

  publishMarkers(s->markers);
}

void RvizMarkerPublisher::showMarkersInfo(const std::string &ns) const
{
  std::map<std::string, MarkersNs>::const_iterator it = ns_map.find(ns);
  if (it == ns_map.end())
  {
    if (enable_topic_not_exist_warnings) ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Marker namespace " << ns << " does not exist.");
    return;
  }

  const MarkersNs *s = &(it->second);
  std::cerr << "================================\n";
  std::cerr << "namespace: " << s->ns << "\n"
            << "ptr: " << s << "\n"
            << "==> markers: \n";
  for (const visualization_msgs::Marker &marker : s->markers.markers)
  {
    std::cerr << "+ id:" << marker.id << ", action:" << marker.action << ", ns: " << marker.ns << "\n";
  }
  std::cerr << "================================\n";

}

void RvizMarkerPublisher::showAllMarkersInfo() const
{
  for (const std::string &ns : getNamespaces() ) showMarkersInfo(ns);
}

void RvizMarkerPublisher::deleteMarkers(const std::string &ns)
{
  std::map<std::string, MarkersNs>::iterator it = ns_map.find(ns);
  if (it == ns_map.end())
  {
    if (enable_topic_not_exist_warnings) ROS_WARN_STREAM_NAMED(LOGNAME, RvizMarkerPublisher_fun_ << "Marker namespace " << ns << " does not exist.");
    return;
  }

  std::vector<visualization_msgs::Marker> &markers = it->second.markers.markers;
  for (int i=0; i<markers.size(); i++) markers[i].action = visualization_msgs::Marker::DELETE;

  publishMarkers(it->second.markers);
  ns_map.erase(it);
}

bool RvizMarkerPublisher::namespaceExists(const std::string &ns) const
{
  std::map<std::string, MarkersNs>::const_iterator it = ns_map.find(ns);
  return it != ns_map.end();
}

void RvizMarkerPublisher::deleteAllMarkers()
{
  for (std::string &ns : getNamespaces()) deleteMarkers(ns);
}

std::vector<std::string> RvizMarkerPublisher::getNamespaces() const
{
  std::vector<std::string> ns_array;
  for (std::map<std::string, MarkersNs>::const_iterator it=ns_map.begin(); it!=ns_map.end(); it++) ns_array.push_back(it->first);
  return ns_array;
}

void RvizMarkerPublisher::drawnow()
{
  if (!rviz_pub.getNumSubscribers())
    ROS_WARN_STREAM_NAMED(LOGNAME, "No subscribers connected to topic '" << rviz_pub.getTopic() << "'. Published visual messages will be lost.");

  // for (int i=0; i<markers_to_pub.markers.size(); i++)
  // {
  //   visualization_msgs::Marker &marker = markers_to_pub.markers[i];
  //   std::cerr << "marker: ns:" << marker.ns << ", id:" << marker.id << ", action:" << marker.action << "\n";
  // }
  // std::cerr << "-------------------------------------\n";
  if (!markers_to_pub.markers.empty())
  {
    rviz_pub.publish(markers_to_pub);
    markers_to_pub.markers.clear();
  }
}

void RvizMarkerPublisher::appendMarkers(visualization_msgs::MarkerArray *m, const visualization_msgs::MarkerArray &m2)
{
  int n2 = m2.markers.size();
  int n = m->markers.size();
  m->markers.resize(n+n2);
  for (int i=0; i<n2; i++) m->markers[n+i] = m2.markers[i];
}

} // namespace rviz_

} // namespace as64_
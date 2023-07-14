#ifndef AS64_RVIZ_MARKER_PUBLISHER_H
#define AS64_RVIZ_MARKER_PUBLISHER_H

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <armadillo>

namespace as64_
{

namespace rviz_
{

class RvizMarkerPublisher; // forward decleration

// ================================================
// ===============  CLASS Color  ==================
// ================================================

class Color
{
public:
  Color(double r, double g, double b, double alpha=1.0);

  // ------- Default colors ---------
  static Color RED;
  static Color GREEN;
  static Color BLUE;
  static Color CYAN;
  static Color MAGENTA;

  static Color BROWN;
  static Color ORANGE;
  static Color YELLOW;
  static Color PURPLE;
  static Color PINK;

  static Color BLACK;
  static Color WHITE;
  
  static Color GREY;
  static Color DARK_GREY;
  static Color LIME_GREEN;

  static Color TRANSLUCENT;
  static Color TRANSLUCENT_LIGHT;
  static Color TRANSLUCENT_DARK;

protected:

  friend RvizMarkerPublisher;

  void setColorParam(double c_ref, double *c);

  double r_;
  double g_;
  double b_;
  double alpha_;
};

// ================================================
// ===============  CLASS Scale  ==================
// ================================================

class Scale
{
public:
  Scale(double scale);

  double operator()() { return scale_; }

  static Scale XXXXSMALL;
  static Scale XXXSMALL;
  static Scale XXSMALL;
  static Scale XSMALL;
  static Scale SMALL;
  static Scale MEDIUM;
  static Scale LARGE;
  static Scale XLARGE;
  static Scale XXLARGE;
  static Scale XXXLARGE;
  static Scale XXXXLARGE;

protected:

  friend RvizMarkerPublisher;
  double scale_;
};

// ==============================================================
// ===============  CLASS RvizMarkerPublisher  ==================
// ==============================================================

class RvizMarkerPublisher
{
public:

  struct MarkersNs
  {
    MarkersNs(const std::string &name_space=""): ns(name_space) {}

    std::string ns;
    visualization_msgs::MarkerArray markers;
    int id = 0; ///< contains the id that will be assigned to the next marker created in this namespace
  };

  RvizMarkerPublisher(const std::string &marker_array_topic, const std::string &base_frame);

  void setBaseFrame(const std::string &base_frame);

  void setGlobalScale(double gb_scale);
  double getGlobalScale() const;

  void setFrameLocking(bool set);

  void publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, const std::string &ns);
  void publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale scale, const std::string &ns);
  void publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale line_width, rviz_::Scale axis_length, const std::string &ns);
  void publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale scale, const std::vector<rviz_::Color> &axes_color, const std::string &ns);
  void publishFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale line_width, rviz_::Scale axis_length, const std::vector<rviz_::Color> &axes_color, const std::string &ns);
  
  void publishPath(const arma::mat &path, rviz_::Color color, rviz_::Scale line_width, const std::string &ns);
  void publishPath(const std::vector<Eigen::Vector3d> &path, rviz_::Color color, rviz_::Scale line_width, const std::string &ns);

  void publishOrientedPath(const arma::mat &path, const arma::mat &orient, unsigned n_frames, rviz_::Color color, rviz_::Scale line_width, rviz_::Scale orient_frame_scale, const std::string &ns);
  void publishOrientedPath(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Quaterniond> &orient, unsigned n_frames, rviz_::Color color, rviz_::Scale line_width, rviz_::Scale orient_frame_scale, const std::string &ns);
  void publishOrientedPath(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Quaterniond> &orient, const std::vector<unsigned> orient_ind, rviz_::Color color, rviz_::Scale line_width, rviz_::Scale orient_frame_scale, const std::string &ns);

  void publishCapsule(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns);
  void publishCapsule(const Eigen::Vector3d &center, const Eigen::Vector3d &z_ax, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns);
  void publishCapsule(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, rviz_::Scale radius, rviz_::Color color, const std::string &ns);

  void publishCylinder(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns);
  void publishCylinder(const Eigen::Vector3d &center, const Eigen::Vector3d &z_ax, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, const std::string &ns);
  void publishCylinder(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, rviz_::Scale radius, rviz_::Color color, const std::string &ns);

  void publishSphere(const Eigen::Vector3d &center, rviz_::Scale radius, rviz_::Color color, const std::string &ns);
  void publishSphere(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale radius, rviz_::Color color, const std::string &ns);
  void publishSphere(const Eigen::Vector3d &center, const Eigen::Vector3d &z_ax, rviz_::Scale radius, rviz_::Color color, const std::string &ns);

  void publishEllipsoid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, std::array<rviz_::Scale, 3> axes_len, rviz_::Color color, const std::string &ns);
  void publishEllipsoid(const Eigen::Vector3d &center, const Eigen::Matrix3d &Sigma, rviz_::Color color, const std::string &ns);

  void publishCuboid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale x_len, rviz_::Scale y_len, rviz_::Scale z_len, rviz_::Color color, const std::string &ns);
 
  void publishPlane(const Eigen::Vector3d &center, const Eigen::Vector3d &normal, rviz_::Scale width, rviz_::Scale height, rviz_::Color color, const std::string &ns);

  void transformMarkers(const Eigen::Isometry3d &tf, const std::string &ns);

  void deleteMarkers(const std::string &ns);

  void deleteAllMarkers();

  std::vector<std::string> getNamespaces() const;

  void drawnow();

  bool waitForSubscriber(double wait_time, bool blocking = false);

  static Eigen::Quaterniond directionToOrient(const Eigen::Vector3d &direction, const Eigen::Vector3d &ref_direction=Eigen::Vector3d(0,0,1));

  void showMarkersInfo(const std::string &ns) const;

  void showAllMarkersInfo() const;

  bool namespaceExists(const std::string &ns) const;

  std::string getMarkerPublishTopic() const { return marker_array_topic_; }
  std::string getBaseFrame() const { return base_frame_; }

  bool enable_topic_not_exist_warnings = true;

protected:

  static std::vector<unsigned> getEquidistantPathPointIndices(const std::vector<Eigen::Vector3d> &path, int n_ind);

  static void appendMarkers(visualization_msgs::MarkerArray *m, const visualization_msgs::MarkerArray &m2);

  visualization_msgs::Marker createCuboid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale x_len, rviz_::Scale y_len, rviz_::Scale z_len, rviz_::Color color, MarkersNs *s);

  visualization_msgs::MarkerArray createCapsuleMarker(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, MarkersNs *s);

  visualization_msgs::MarkerArray createFrameMarker(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, rviz_::Scale line_width, rviz_::Scale axis_length,  const std::vector<rviz_::Color> &axes_color, MarkersNs *s);

  visualization_msgs::Marker createCylinderMarker(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, rviz_::Scale radius, rviz_::Color color, MarkersNs *s);
  visualization_msgs::Marker createCylinderMarker(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale height, rviz_::Scale radius, rviz_::Color color, MarkersNs *s);

  visualization_msgs::Marker createSphere(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, rviz_::Scale radius, rviz_::Color color, MarkersNs *s);

  visualization_msgs::Marker createEllipsoid(const Eigen::Vector3d &center, const Eigen::Quaterniond &orient, std::array<rviz_::Scale, 3> axes_len, rviz_::Color color, MarkersNs *s);
  visualization_msgs::Marker createEllipsoid(const Eigen::Vector3d &center, const Eigen::Matrix3d &Sigma, rviz_::Color color, MarkersNs *s);

  RvizMarkerPublisher::MarkersNs *getMarkersNamespaceStruct(const std::string &ns);

  void publishMarker(const visualization_msgs::Marker &marker);
  void publishMarkers(const visualization_msgs::MarkerArray &markers);

  std::map<std::string, visualization_msgs::MarkerArray> markers_map;

  std::string base_frame_;
  std::string marker_array_topic_;

  ros::Publisher rviz_pub;

  double global_scale_ = 1.0;

  double lifetime_ = 0;

  bool frame_locked_ = true;

  double axis_length_ = 0.1;
  double axis_width_ = 0.022;
  rviz_::Color x_axis_color = rviz_::Color::RED;
  rviz_::Color y_axis_color = rviz_::Color::GREEN;
  rviz_::Color z_axis_color = rviz_::Color::BLUE;

  std::map<std::string, MarkersNs> ns_map;

  visualization_msgs::MarkerArray markers_to_pub;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html

};


} // namespace rviz_

} // namespace as64_

#endif // AS64_RVIZ_MARKER_PUBLISHER_H
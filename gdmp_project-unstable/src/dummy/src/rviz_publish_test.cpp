#include <iostream>
#include <vector>
#include <map>
#include <string>

#include <ros/ros.h>

#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

#include <tf/transform_broadcaster.h>

#include <armadillo>

#include <rviz_lib/rviz_marker_publisher.h>

using namespace as64_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rviz_publish_test");

  ros::NodeHandle nh;

  // tf::TransformBroadcaster tf_pub;
  // tf::StampedTransform tf_transform;
  // tf_transform.stamp_ = ros::Time::now();
  // tf_transform.frame_id_ = "world";
  // tf_transform.child_frame_id_ = "world";
  // tf_transform.setIdentity();


  // urdf::Model urdf_model;
  // if (!urdf_model.initParam("/robot_description"))
  //   throw std::ios_base::failure("Couldn't load urdf model from '/robot_description'...\n");

  // KDL::Tree kdl_tree;
  // kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree);

  // int n_joints = kdl_tree.getNrOfJoints();
  // std::cerr << "n_joints = " << n_joints << "\n";

  // const KDL::SegmentMap seg_map = kdl_tree.getSegments();

  // std::map<std::string, double> joints_maps;

  // for (KDL::SegmentMap::const_iterator it = seg_map.begin(); it!=seg_map.end(); it++)
  // {
  //   // std::string link_name = it->first;
  //   KDL::Segment seg = it->second.segment;
  //   KDL::Joint joint = seg.getJoint();

  //   std::cerr << "seg = " << seg.getName() << ",  joint = " << seg.getJoint().getName() << "\n";

  //   if (seg.getName().compare("world") == 0) continue;
  //   if (joint.getType() == KDL::Joint::None) continue;

  //   joints_maps[seg.getJoint().getName()] = 0.0;
  // }

  // for (auto it = joints_maps.begin(); it!= joints_maps.end(); it++) std::cerr << it->first << ": " << it->second << "\n";

  // robot_state_publisher::RobotStatePublisher rs_pub(kdl_tree);
  
  int n_data = 1000;
  arma::rowvec Time = arma::linspace<arma::rowvec>(0, 1, n_data);
  arma::mat Pos_data(3,n_data);
  Pos_data.row(0) = Time;
  Pos_data.row(1) = arma::sin(2*Time);
  Pos_data.row(2) = arma::cos(Time);

  arma::mat Pos_data2(3,n_data);
  Pos_data2.row(0) = Time;
  Pos_data2.row(1) = 1.2*arma::cos(4*Time);
  Pos_data2.row(2) = 1.3*arma::sin(3*Time);

  arma::mat Pos_data3 = 1.2*arma::sqrt(Pos_data);

  std::vector<Eigen::Vector3d> Pos_data_eig(n_data);
  std::vector<Eigen::Quaterniond> Quat_data_eig(n_data);

  std::vector<Eigen::Vector3d> Pos_data2_eig(n_data);
  std::vector<Eigen::Vector3d> Pos_data3_eig(n_data);
  for (int i=0;i<n_data;i++)
  {
    Eigen::Vector3d q( Pos_data(0,i), Pos_data(1,i), Pos_data(2,i) );
    Quat_data_eig[i] = Eigen::AngleAxisd( q.norm(), q/q.norm() );
    Eigen::Quaterniond( q(0),  q(1),  q(2),  q(3) );
    Pos_data_eig[i] = Eigen::Vector3d( Pos_data(0,i), Pos_data(1,i), Pos_data(2,i) );
    Pos_data2_eig[i] = Eigen::Vector3d( Pos_data2(0,i), Pos_data2(1,i), Pos_data2(2,i) );
    Pos_data3_eig[i] = Eigen::Vector3d( Pos_data3(0,i), Pos_data3(1,i), Pos_data3(2,i) );
  }
  

  rviz_::RvizMarkerPublisher rviz_pub("my_rviz_markers", "world");

  rviz_pub.waitForSubscriber(2);


  rviz_pub.publishCuboid(Eigen::Vector3d(0, 0, 0),  Eigen::Quaterniond(1, 0, 0, 0), 1.0, 1.0, 1.0, rviz_::Color(1,0,0,0.2), "ns15");

  arma::vec ax_len = {0.5, 0.5, 0.5};
  Eigen::Matrix3d Sigma;
  rviz_pub.publishEllipsoid(Eigen::Vector3d(0, 0, 0), Sigma, rviz_::Color(0,0,1,0.5), "ns15");

  rviz_pub.drawnow();

  while (ros::ok());

  // ros::Rate(1).sleep();
  
  // rviz_pub.publishCylinder(Eigen::Vector3d(0.5,0.5,0.1), Eigen::Quaterniond( Eigen::AngleAxisd( 1.57, Eigen::Vector3d(1,0,0) ) ), 0.8, 0.1, rviz_::Color::PURPLE, "ns4");

  // rviz_pub.publishCylinder(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.5,0.5,0.5), 0.1, rviz_::Color::MAGENTA, "ns5");
  // rviz_pub.publishCylinder(Eigen::Vector3d(0.5,0.5,0), Eigen::Vector3d(0,0,0.5), 0.1, rviz_::Color::BLUE, "ns5");
  // rviz_pub.publishCylinder(Eigen::Vector3d(0.25,0.25,0.25), Eigen::Vector3d(0.5,0.5,-0.5), 1.2, 0.1, rviz_::Color(0,0,1,0.4), "ns5");

  // rviz_pub.publishCapsule(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.5,0.5,0.5), 0.1, rviz_::Color::MAGENTA, "ns15");
  // rviz_pub.publishCapsule(Eigen::Vector3d(0.25,0.25,0.25), Eigen::Vector3d(0.5,0.5,-0.5), 1.2, 0.1, rviz_::Color(0,0,1,0.4), "ns15");

  // rviz_pub.publishFrame(Eigen::Vector3d(0,0,0.1), Eigen::Quaterniond(Eigen::AngleAxisd(3.14/4, Eigen::Vector3d(1,0,0))), 0.025, 0.1, "ns15");

  // rviz_pub.publishFrame(Eigen::Vector3d(0,0,0.3), Eigen::Quaterniond(Eigen::AngleAxisd(3.14/4, Eigen::Vector3d(0,0,1))), "ns15");

  // rviz_pub.publishFrame(Eigen::Vector3d(0,0,0.6), Eigen::Quaterniond(Eigen::AngleAxisd(3.14/4, Eigen::Vector3d(0,0,1))), 2, "ns15");

  rviz_pub.publishCapsule(Pos_data_eig[0], Quat_data_eig[0], 0.2, 0.03, rviz_::Color(0,1,1,0.8), "ns15");

  rviz_pub.publishCuboid(Pos_data_eig[0],  Quat_data_eig[0], 0.1, 0.2, 0.02, rviz_::Color(1,0,0,0.15), "ns15");

  rviz_pub.publishCapsule(Pos_data_eig[0], Quat_data_eig[0], 0.2, 0.03, rviz_::Color::RED, "ns2");

  rviz_pub.publishOrientedPath(Pos_data_eig, Quat_data_eig, 10, rviz_::Color::BROWN, 0.02, 1.2, "ns2");

  // Eigen::Quaterniond orient = Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ());
  // rviz_pub.publishCuboid(Eigen::Vector3d(0, 0, 0), orient, 0.8, 0.3, 0.1, rviz_::Color::MAGENTA, "ns10");

  // arma::rowvec x_lim = {0.1, 0.5};
  // arma::rowvec y_lim = {0.2, 0.7};
  // arma::rowvec z_lim = {0, 0.6};

  // Eigen::Vector3d center( (x_lim(1)+x_lim(0))/2 , (y_lim(1)+y_lim(0))/2 , (z_lim(1)+z_lim(0))/2 );
  // rviz_pub.publishCuboid(center + Eigen::Vector3d(0.6,0.6,0), Eigen::Quaterniond(1,0,0,0), x_lim(1)-x_lim(0), y_lim(1)-y_lim(0), z_lim(1)-z_lim(0), rviz_::Color(1,0,0,0.15), "ns10");

  // rviz_pub.publishPlane(Eigen::Vector3d(center(0), center(1), z_lim(0)), Eigen::Vector3d(0, 0, 1), x_lim(1)-x_lim(0), y_lim(1)-y_lim(0), rviz_::Color(1,0,0,0.15), "ns10");
  // rviz_pub.publishPlane(Eigen::Vector3d(center(0), center(1), z_lim(1)), Eigen::Vector3d(0, 0, 1), x_lim(1)-x_lim(0), y_lim(1)-y_lim(0), rviz_::Color(1,0,0,0.15), "ns10");

  // rviz_pub.publishPlane(Eigen::Vector3d(x_lim(0), center(1), center(2)), Eigen::Vector3d(1, 0, 0), z_lim(1)-z_lim(0), y_lim(1)-y_lim(0), rviz_::Color(1,0,0,0.15), "ns10");
  // rviz_pub.publishPlane(Eigen::Vector3d(x_lim(1), center(1), center(2)), Eigen::Vector3d(1, 0, 0), z_lim(1)-z_lim(0), y_lim(1)-y_lim(0), rviz_::Color(1,0,0,0.15), "ns10");

  // rviz_pub.publishPlane(Eigen::Vector3d(center(0), y_lim(0), center(2)), Eigen::Vector3d(0, 1, 0), x_lim(1)-x_lim(0), z_lim(1)-z_lim(0), rviz_::Color(1,0,0,0.15), "ns10");
  // rviz_pub.publishPlane(Eigen::Vector3d(center(0), y_lim(1), center(2)), Eigen::Vector3d(0, 1, 0), x_lim(1)-x_lim(0), z_lim(1)-z_lim(0), rviz_::Color(1,0,0,0.15), "ns10");


  // rviz_pub.publishPlane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1), 0.8, 0.3, rviz_::Color(1,0,0,0.5), "ns10");

  // rviz_pub.publishOrientedPath(Pos_data_eig, Quat_data_eig, {120, 250, 600}, rviz_::Color::BROWN, 0.02, 1.2, "ns1");

  // rviz_pub.publishSphere(Eigen::Vector3d(-0.8, 1, -0.4), 0.8, rviz_::Color::CYAN, "ns10");
  // rviz_pub.publishSphere(Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1,1,0), 0.6, rviz_::Color(1,0,1), "ns10");
  
  rviz_pub.drawnow();

  // rviz_pub.deleteMarkers("ns5");

  // rviz_pub.drawnow();

  int i = 0;

  ros::Rate loop_rate(200);
  while (ros::ok())
  {
    i++;

    if (i == Pos_data_eig.size()) i=0;

    // rviz_pub.showAllMarkersInfo();

    // if (rviz_pub.namespaceExists("ns1"))
    // {
    //   std::cerr << "Deleting 'ns1'...";
    //   rviz_pub.deleteMarkers("ns1");
    //   std::cerr << "[DONE]\n";
    // }

    rviz_pub.deleteMarkers("ns15");
    // rviz_pub.publishCapsule(Pos_data_eig[i], Quat_data_eig[i], 0.2, 0.03, rviz_::Color(0,1,1,0.8), "ns15");
    rviz_pub.publishEllipsoid(Pos_data_eig[i], Quat_data_eig[i], {0.2, 0.05, 0.05}, rviz_::Color(0,1,1,0.8), "ns15");
    rviz_pub.publishCuboid(Pos_data_eig[i],  Quat_data_eig[i], 0.1, 0.2, 0.02, rviz_::Color(1,0,0,0.15), "ns15");
    rviz_pub.drawnow();

    // rviz_pub.showMarkersInfo("ns2");
    // rviz_pub.showMarkersInfo("ns15");

    // while (ros::ok());

    // if (i==100)
    // {
    //   std::cerr << "Publishing ns 1...\n";
    //   rviz_pub.publishPath(Pos_data, rviz_::Color::ORANGE, rviz_::Scale::MEDIUM, "ns1");
    //   rviz_pub.drawnow();
    // }

    // if (i==200)
    // {
    //   std::cerr << "Publishing ns 2...\n";
    //   // rviz_pub.publishPath(Pos_data2, rviz_::Color::MAGENTA, rviz_::Scale::MEDIUM, "ns2");
    //   // rviz_pub.publishPath(Pos_data3, rviz_::Color::RED, rviz_::Scale::LARGE, "ns2");
    //   rviz_pub.publishPath(Pos_data2_eig, rviz_::Color::MAGENTA, rviz_::Scale::MEDIUM, "ns9");
    //   rviz_pub.publishPath(Pos_data3_eig, rviz_::Color::RED, rviz_::Scale::LARGE, "ns9");
    //   rviz_pub.drawnow();
    // }


    // if (i==400)
    // {
    //   std::cerr << "Removing ns 1...\n";
    //   rviz_pub.deleteMarkers("ns1");
    //   rviz_pub.drawnow();
    // }

    // if (i==600)
    // {
    //   std::cerr << "Removing ns 2...\n";
    //   rviz_pub.publishCylinder(Eigen::Vector3d(0.5,-1,0), Eigen::Quaterniond(0.2,0.8,0.3,-0.4), 0.8, 0.1, rviz_::Color::LIME_GREEN, "ns1");
    //   rviz_pub.publishCylinder(Eigen::Vector3d(0,-0.8,0), Eigen::Quaterniond(1,0,0,0), 0.8, 0.1, rviz_::Color::PURPLE, "ns4");
    //   rviz_pub.deleteMarkers("ns2");
    //   rviz_pub.drawnow();
    // }

    // if (i == 700)
    // {
    //   std::cerr << "Removing all ns...\n";
    //   rviz_pub.deleteAllMarkers();
    //   rviz_pub.drawnow();
    //   i = 0;
    // }

     
    // rs_pub.publishTransforms(joints_maps, ros::Time::now(), "");
    // rs_pub.publishFixedTransforms("", true);
    loop_rate.sleep();
  }

  return 0;
}
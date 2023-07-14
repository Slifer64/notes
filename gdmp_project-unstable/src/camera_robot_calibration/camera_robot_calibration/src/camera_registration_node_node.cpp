#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <ros/package.h>
#include <armadillo>


int main(int argc, char **argv)
{
  ros::init(argc,argv, "camera_registration_tf");
  ros::NodeHandle node;  
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;

  
  std::string DIR = ros::package::getPath("camera_registration");
  std::string filename,PATH;

  filename = "transform_robot_tags";
  PATH = DIR + "/calibration/" + filename + ".dat";


  ifstream myfile(PATH,ios::in);


  int tags_num = 4;

  arma::fmat camera_t = arma::zeros<arma::fmat>(3,tags_num);
  arma::fmat robot_t = arma::zeros<arma::fmat>(3,tags_num);  
  arma::fmat transform_a = arma::zeros<arma::fmat>(3,tags_num);  

  int tags = 0 ;
  
  for(int i=0;i<tags_num;i++){

  while(tags < i + 1 ){
      try{
        transformStamped = tfBuffer.lookupTransform("camera", "tag_" + std::to_string(i),ros::Time(0));
        tags++;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();    
        continue;
      }
  }

    camera_t(0,i)  = transformStamped.transform.translation.x;
    camera_t(1,i)  = transformStamped.transform.translation.y;
    camera_t(2,i)  = transformStamped.transform.translation.z;

  }

  std::cout << "tags: " << camera_t << std::endl ;


  arma::fvec centroid_a = arma::zeros<arma::fvec>(3);
  arma::fvec centroid_b = arma::zeros<arma::fvec>(3);
   
  centroid_a = arma::sum(camera_t,1)/camera_t.n_cols;
  centroid_b = arma::sum(robot_t,1)/robot_t.n_cols;  

  arma::fmat t1 = arma::zeros<arma::fmat>(3,tags_num);
  arma::fmat t2 = arma::zeros<arma::fmat>(3,tags_num);

  t1.each_col() -= centroid_a;
  t2.each_col() -= centroid_b;

  arma::fmat H = t1*t2.t() ;

  arma::fmat U;
  arma::fvec S;
  arma::fmat V;
  arma::svd(U, S, V, H,"std");

  arma::fmat R = V*U.t();

  if(arma::det(R) < 0 ){
  arma::svd(U, S, V, R,"std");
  V.submat(1,3,3,3) = - V.submat(1,3,3,3) ;
  R = V*U.t();
  }

  arma::fvec t = centroid_a - R * centroid_b ;  

  transform_a = R * camera_t ;
  transform_a.each_col() += t ;

  arma::fmat e = transform_a-robot_t;
  double er =0 ;
  for(int i=0;i<e.n_cols;i++){
    er += arma::norm(e.col(i))/e.n_cols;
  }

  std::cout << " Calibration err: " << er << std::endl;

  std::string filename = "transform_camera_robot";
	std::string PATH = DIR + "/calibration/" + filename + ".dat";
  
  
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  std::string static_camera_name = "camera";

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";

  static_transformStamped.child_frame_id = static_camera_name;
  static_transformStamped.transform.translation.x = atof(argv[2]);
  static_transformStamped.transform.translation.y = atof(argv[3]);
  static_transformStamped.transform.translation.z = atof(argv[4]);
  tf2::Quaternion quat;
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
 
  ros::spin();
  return 0;
};

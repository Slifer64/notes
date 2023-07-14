#include <robot_camera_tf_publisher/controller.h>

#include <iostream>
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_camera_tf_publisher" );

  std::shared_ptr<Controller> main_controller(new Controller());

  main_controller->exec();

  return 0;
}

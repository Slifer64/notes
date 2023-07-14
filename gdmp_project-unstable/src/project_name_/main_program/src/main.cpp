#include <iostream>
#include <memory>
#include <ros/ros.h>

#include <main_controller/main_controller.h>
#include <gmp_mpc_controller/gmp_mpc_controller.h>
#include <train_controller/train_controller.h>
#include <admittance_controller/controller.h>
#include <online_adapt_controller/controller.h>
#include <im_traj_demo_controller/controller.h>
#include <grasp_obj_controller/controller.h>

int main(int argc, char **argv)
{
  std::string ros_init_name = "$project_name$";
  if (argc > 1) ros_init_name += argv[1];

  ros::init(argc, argv, ros_init_name );
  ros::NodeHandle nh("~");
  bool ctrl_on;

  std::shared_ptr<MainController> main_controller(new MainController());
  if (nh.getParam("train_controller", ctrl_on) && ctrl_on) main_controller->addController(new TrainController(main_controller.get(), "train controller"));
  if (nh.getParam("gmp_mpc_controller", ctrl_on) && ctrl_on) main_controller->addController(new GmpMpcController(main_controller.get(), "gmp-mpc test"));
  if (nh.getParam("admittance_controller", ctrl_on) && ctrl_on) main_controller->addController(new AdmittanceController(main_controller.get(), "variable admittance"));
  if (nh.getParam("online_adapt_controller", ctrl_on) && ctrl_on) main_controller->addController(new OnlineAdaptController(main_controller.get(), "online adaptation"));
  if (nh.getParam("im_traj_demo_controller", ctrl_on) && ctrl_on) main_controller->addController(new ImTrajDemoController(main_controller.get(), "Image-Traj Demo"));
  if (nh.getParam("grasp_obj_controller", ctrl_on) && ctrl_on) main_controller->addController(new GraspObjController(main_controller.get(), "grasp object ctrl"));
  main_controller->exec();

  ros::shutdown();

  return 0;
}

#ifndef $_PROJECT_384$_GRASP_OBJ_CONTROLLER_H
#define $_PROJECT_384$_GRASP_OBJ_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <grasp_obj_controller/gui.h>

#include <thread_lib/semaphore.h>
#include <thread_lib/mtx_var.h>

#include <robot_wrapper/robot.h>
#include <main_controller/main_controller.h>
#include <main_controller/controller.h>

#include <rviz_lib/rviz_marker_publisher.h>

#include <gmp_lib/GMP/GMP.h>
#include <gmp_lib/GMP/GMPo.h>
#include <math_lib/math_lib.h>

#include <ros_lib/image_reader.h>
#include <ros_lib/camera_params.h>
#include <ros_lib/transform_reader.h>

#include <QPushButton>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <grasp_obj_controller/ImageToMPAction.h>

using namespace as64_;

class GraspObjController : public Controller
{
public:

  GraspObjController(MainController *main_ctrl, const std::string &ctrl_name);
  ~GraspObjController();

  QPushButton *createGui(MainWindow *parent) override;

  std::string getDefaultPath() const { return default_data_path; }

public:

  gmp_::GMP::Ptr mp;
  gmp_::GMPo::Ptr mp_o;

  ros::NodeHandle nh;

  std::shared_ptr<ros_lib_::ImageReader> img_reader;

  ros::AsyncSpinner async_spinner;

  GraspObjGui *gui;

  void startExec();
  ExecResultMsg stopExec();
  void runModel(bool reverse=false);

  ExecResultMsg moveToStartPose();

  void readImage();
  void retract();

  ExecResultMsg record();

  void viewExecPath(bool view);

  ExecResultMsg loadParams(const std::string &path="");

  struct
  {
    unsigned N_kernels;
    std::string image_topic;
    std::string depth_image_topic;
    std::string cam_info_topic;
    std::string robot_cam_tf_topic;
    arma::vec task_n;
    arma::vec task_p0;
    std::string resnet_action_name;
    double Tf_per_meter;
    double retract_z_offset;
  } params;

  struct
  {
    bool active;
    arma::vec force_scale;
    double gain;
  } phase_stop;

  struct
  {
    double filt;
    arma::vec deadzone;
    arma::uvec enabled_dofs;
  } force_params;

  struct
  {
    std::vector<double> K;
    std::vector<double> D;
    std::vector<double> M;
  } dmp_params;

  struct
  {
    double open_angle;
    double close_angle;
    double max_force;
  } gripper;

  double fx, fy;
  double cx, cy;
  unsigned img_height, img_width;
  void readCameraIntrinsics();

  std::shared_ptr<ros_lib_::TransformReader> robot_cam_tf;

  thr_::MtxVar<bool> exec_on;
  thr_::Semaphore exec_stop_sem;

  std::shared_ptr<rviz_::RvizMarkerPublisher> rviz_pub;
  std::string base_link; // used for publishing to rviz

  std::string default_data_path;

  double getExecPathLength(unsigned n_points=100) const;

  std::shared_ptr<actionlib::SimpleActionClient<grasp_obj_controller::ImageToMPAction>> action_client;
  void imageToMP_doneCb(const actionlib::SimpleClientGoalState& state, const grasp_obj_controller::ImageToMPResultConstPtr& result);
  void imageToMP_activeCb();
  void imageToMP_feedbackCb(const grasp_obj_controller::ImageToMPFeedbackConstPtr& feedback);

  double s;
  arma::vec Pg, Qg;
  arma::vec P0, Q0;

  std::vector<cv::Mat> rgb_images;
  std::vector<cv::Mat> depth_images;
  std::vector<arma::mat> mp_weights;

  struct Record_
  {
    int sample_count = 1;

    cv::Mat img;
    cv::Mat depth;
    arma::mat mp_weights;
    gmp_::GMP *mp;
    gmp_::GMPo *mp_o;

    bool save(const std::string &path, std::string *err_msg=NULL);
  };
  Record_ record_;

};

#endif // $_PROJECT_384$_GRASP_OBJ_CONTROLLER_H

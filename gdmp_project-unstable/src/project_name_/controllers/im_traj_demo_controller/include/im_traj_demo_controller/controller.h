#ifndef $_PROJECT_384$_IM_TRAJ_DEMO_CONTROLLER_H
#define $_PROJECT_384$_IM_TRAJ_DEMO_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <im_traj_demo_controller/gui.h>

#include <thread_lib/semaphore.h>
#include <thread_lib/mtx_var.h>

#include <robot_wrapper/robot.h>
#include <main_controller/main_controller.h>
#include <main_controller/controller.h>

#include <rviz_lib/rviz_marker_publisher.h>

#include <gmp_lib/GMP/GMP.h>
#include <gmp_lib/GMP/GMPo.h>
#include <math_lib/math_lib.h>

#include <ros_lib/move_to_joints_pos_client.h>
#include <ros_lib/image_reader.h>
#include <ros_lib/camera_params.h>
#include <ros_lib/transform_reader.h>

#include <QPushButton>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace as64_;

namespace im_traj_demo_
{
  struct LogData
  {
    arma::rowvec Time;
    arma::mat P_data;
    arma::mat Q_data;

    LogData() {}

    void clear()
    {
      Time.clear();
      P_data.clear();
      Q_data.clear();
    }

    int size() const { return Time.size(); }

    void add(double t, const arma::vec &pos, const arma::vec &quat)
    {
      Time = arma::join_horiz(Time, arma::vec({t}));
      P_data = arma::join_horiz(P_data, pos);
      Q_data = arma::join_horiz(Q_data, quat);
    }

    void trim(double pos_thres=0.005, double orient_thres=0.05)
    {
      if (size() == 0)
      {
        std::cerr << "\033[1m" << "\033[33m" << "[WARNING]: " << "The data are empty...\n" << "\033[0m";
        return;
      }

      int n_data = Time.size();
      int j1 = 0;
      int j2 = n_data-1;

      for (int j=0; j<n_data-1; j++)
      {
        double dt = Time(j+1) - Time(j);
        if ( arma::norm( (P_data.col(j+1) - P_data.col(0)) /*/ dt*/ ) >= pos_thres  ||  
            arma::norm( math_::quatLog(math_::quatDiff(Q_data.col(j+1), Q_data.col(0))) /*/ dt*/ ) >= orient_thres )
        { j1 = j; break; }
      }

      for (int j=n_data-2; j>j1; j--)
      {
        double dt = Time(j+1) - Time(j);
        if ( arma::norm( (P_data.col(n_data-1) - P_data.col(j)) /*/ dt*/ ) >= pos_thres  ||  
            arma::norm( math_::quatLog(math_::quatDiff(Q_data.col(n_data-1), Q_data.col(j))) /*/ dt*/ ) >= orient_thres )
        { j2 = j+1; break; }
      }

      std::cerr << "====== Trim demo =======\n";
      std::cerr << "Initial: [" << 0 << ", " << n_data-1 << "]\n";
      std::cerr << "Final: [" << j1 << ", " << j2 << "]\n";

      Time = Time.subvec(j1, j2) - Time(j1);
      P_data = P_data.cols(j1,j2);
      Q_data = Q_data.cols(j1,j2);
    }
  };
}

class ImTrajDemoController : public Controller
{
public:

  ImTrajDemoController(MainController *main_ctrl, const std::string &ctrl_name);
  ~ImTrajDemoController();

  QPushButton *createGui(MainWindow *parent) override;

  std::string getDefaultPath() const { return default_data_path; }

public:

  im_traj_demo_::LogData log;

  gmp_::GMP::Ptr mp;
  gmp_::GMPo::Ptr mp_o;
  bool is_demo_captured = false;

  bool is_image_captured = false;

  ros::NodeHandle nh;

  std::shared_ptr<ros_lib_::ImageReader> img_reader;

  ros::AsyncSpinner async_spinner;

  ImTragDemoGui *gui;

  void startDemoRec();
  ExecResultMsg stopDemoRec();
  void pathTeaching();

  void captureImage();
  void captureMultiImages();
  ExecResultMsg saveDemo();

  bool isData() const { return log.size() != 0; }

  void viewLearnedPath(bool view);

  void clearImages();

  void loadParams(std::string path="");
  struct
  {
    unsigned N_kernels;
    std::string image_topic;
    std::string depth_image_topic;
    std::string cam_info_topic;
    std::string robot_cam_tf_topic;
    std::string move2jpos_topic;
    std::vector<std::vector<double>> im_capt_joint_pos;
  } params;

  arma::mat Cam_Proj_mat;
  void readCameraIntrinsics();

  std::shared_ptr<ros_lib_::TransformReader> robot_cam_tf;
  
  std::vector<cv::Mat> images;
  std::vector<cv::Mat> depth_images;
  std::vector<arma::vec> robot_cam_tf_data;

  thr_::MtxVar<bool> exec_on;
  thr_::Semaphore exec_stop_sem;

  std::shared_ptr<rviz_::RvizMarkerPublisher> rviz_pub;
  std::string base_link; // used for publishing to rviz

  // model params
  thr_::MtxVar<unsigned> N_kernels;
  thr_::MtxVar<std::string> train_method;

  std::string default_data_path;

  std::shared_ptr<MoveToJointsPosActionClient> move2jpos_client;
};

#endif // $_PROJECT_384$_IM_TRAJ_DEMO_CONTROLLER_H

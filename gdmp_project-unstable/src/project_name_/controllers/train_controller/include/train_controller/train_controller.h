#ifndef $_PROJECT_384$_TRAIN_CONTROLLER_H
#define $_PROJECT_384$_TRAIN_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <train_controller/utils/demo_data.h>
#include <train_controller/utils/ref_pose_publisher.h>
#include <train_controller/train_gui.h>

#include <robot_wrapper/robot.h>
#include <gmp_model/model_wrapper.h>
#include <main_controller/main_controller.h>
#include <main_controller/controller.h>

#include <rviz_lib/rviz_marker_publisher.h>

#include <std_msgs/Int16.h>

#include <QPushButton>

using namespace as64_;

class TrainController : public Controller
{
public:
  enum RosAction
  {
    EXEC_TRAJ = 0,
    EXEC_REVESE_TRAJ = 1,
    START_TRAIN = 2,
    STOP_TRAIN = 3
  };

  class FextFilt
  {
    public:
      FextFilt(double a_f = 0.005)
      {
        reset();
        this->a_f = a_f;
      }

      void reset() { Fext_prev = arma::vec().zeros(6); }

      arma::vec operator()(const arma::vec &Fext)
      {
        Fext_prev = (1-a_f)*Fext_prev + a_f*Fext;
        return Fext_prev;
      }

    private:
      arma::vec Fext_prev;
      double a_f;
  };

  TrainController(MainController *main_ctrl, const std::string &ctrl_name);
  ~TrainController();

  QPushButton *createGui(MainWindow *parent) override;

  std::string getDefaultPath() const { return default_train_data_path; }

public:

  ros::NodeHandle node;
  ros::Subscriber ros_sub;
  void rosSubCallback(const std_msgs::Int16::ConstPtr& msg);

  ModelWrapper model;

  TrainWin *gui;

  // ========================================
  // ========   Model Train  ================
  bool isTrainData() const { return !demo_data.empty(); }
  bool isModelTrained() const 
  { return (model() && model->isTrained()); }

  void trainPhaseChanged(TrainPhase phase);
  void setPathTeachMethod(enum PathTeachMethod method);

  void setPhaseDamping(double Dx) { this->Dx.set(Dx); }
  void setFvScaling(double a_fv) { this->a_fv.set(a_fv); }
  void setFvFilter(double a_filt) { this->a_filt.set(a_filt); }
  void setNkernels(unsigned N_kernels) { this->N_kernels.set(N_kernels); }
  void setTrainMethod(const std::string &train_method) { this->train_method.set(train_method); }

  void setModelType(const std::string &train_method);

  void setTargetPose(const arma::vec &target);
  arma::vec getTargetPose() const;
  void setCurrentPoseAsTarget();

  void startDemoRec();
  ExecResultMsg stopDemoRec();
  void pathTeaching();
  void velocityTeaching();
  void adaptLearnedTrajectory();

  void executeTrainTrajectory(bool reverse = false);

  void executeLearnedTrajectory(double tau, bool reverse = false);

  void setStartPoseFromTrainData();
  void setTargetPoseFromModel();

  ExecResultMsg equalizeVelocityProfile();
  void setEqualizationNominalVelocity(double nom_vel) { eq_vel_prof_nom_vel.set(nom_vel); }
  void setEqualizationIterations(unsigned iters) { eq_vel_prof_iters.set(iters); }
  void plotVelocityProfile();
  thr_::MtxVar<double> eq_vel_prof_nom_vel;
  thr_::MtxVar<double> eq_vel_prof_iters;


  void plotTrainData();
  ExecResultMsg loadTrainData(const std::string &path);
  ExecResultMsg saveTrainData(const std::string &path);
  void clearDemoData();
  ExecResultMsg trainModel();
  ExecResultMsg trimDemoData(double pos_thres, double orient_thres);
  ExecResultMsg undoTrimDemoData();
  void autoTrimDemoData();

  ExecResultMsg removeStopsFromDemo(double vel_thres);

  double getProgress() const { return x_progress; }

  void plotModelSimData();

  void viewLearnedPath(bool view);
  void viewTargetPose(bool view);

  // Initial and final values of the last executed trajectory
  arma::vec P0_exec;
  arma::vec Q0_exec;

  thr_::MtxVar<bool> enable_logging;
  thr_::MtxVar<bool> autotrim_train_data;
  thr_::MtxVar<bool> phase_ff;
  thr_::MtxVar<bool> enable_online_adaptation;
  thr_::MtxVar<bool> display_ref_frame;
  thr_::MtxVar<bool> exec_on;
  thr_::Semaphore exec_stop_sem;

  // velocity-teach params
  thr_::MtxVar<double> Dx;
  thr_::MtxVar<double> a_fv;
  thr_::MtxVar<double> a_filt;


  std::shared_ptr<RefPosePublisher> ref_pose_pub;
  std::shared_ptr<rviz_::RvizMarkerPublisher> rviz_pub;

  // model params
  thr_::MtxVar<unsigned> N_kernels;
  thr_::MtxVar<std::string> train_method;

  // demo data
  DemoData demo_data;

  arma::rowvec x_train_data;

  std::string default_train_data_path;

  double x_progress;

  arma::vec target_pose;

  std::string base_link; // used for publishing to rviz

  TrainController::FextFilt fext_filt;

};

#endif // $_PROJECT_384$_TRAIN_CONTROLLER_H

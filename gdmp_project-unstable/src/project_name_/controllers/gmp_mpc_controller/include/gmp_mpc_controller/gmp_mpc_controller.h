#ifndef $_PROJECT_384$_GMP_MPC_CONTROLLER_H
#define $_PROJECT_384$_GMP_MPC_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <list>
#include <memory>
#include <mutex>
#include <functional>
#include <armadillo>
#include <Eigen/Dense>

#include <gmp_lib/GMP/GMP_MPC.h>
#include <gmp_lib/GMP/GMP_Opt.h>

#include <io_lib/file_io.h>

#include <rviz_lib/rviz_marker_publisher.h>
#include <thread_lib/semaphore.h>

#include <main_controller/main_controller.h>
#include <main_controller/controller.h>
#include <main_controller/utils.h>

#include <robot_wrapper/robot.h>

#include <gmp_mpc_controller/gmp_mpc_gui.h>

#include <apriltag_ros/apriltag_listener.h>

#include <gmp_mpc_controller/experiment/experiment.h>

using namespace as64_;

struct MpcSettings
{
  int N_horizon;
  double pred_time_step;
  int N_kernels;
  double kernels_std_scaling;

  double kernels_trunc_thres;

  double opt_pos_gain;
  double opt_vel_gain;

  std::array<double,3> slack_gains;
  std::array<double,3> slack_limits;

  arma::vec final_state_err_tol;

  double time_limit;
  unsigned max_iter;
  double abs_tol;
  double rel_tol;
};

class GmpMpcController : public Controller
{
public:

  GmpMpcController(MainController *main_ctrl, const std::string &ctrl_name);
  ~GmpMpcController();

  QPushButton *createGui(MainWindow *parent) override;

  ExecResultMsg loadModel(const std::string &path);
  ExecResultMsg saveModel(const std::string &path);
  ExecResultMsg saveLogData(const std::string &path);
  ExecResultMsg loadParams(const std::string &path);

  void startExec();
  bool stopExec();

  void viewPosBounds(bool view);
  void clearAllMarkers();

  void gotoStartPose();

  std::string getDataPath() const { return data_path; }
  std::string getConfigPath() const { return config_path; }
  std::string getModelPath() const { return model_path; }

  bool isLogData() const { return !mpc_data.isEmpty(); }
  bool isModelLoaded() const;

  ExecResultMsg loadAprilTagListener(const std::string &config_file);
  ExecResultMsg killAprilTagListener();
  void publishOperationTags(const std::string &op_name, bool set);
  std::vector<std::string> op_names;
  std::map<std::string, bool> pub_op_tags_map;
  void publishDetectionsToTf(bool set);


  void viewTargetPose(bool view);

  void setCurrentPoseAsTarget();

  void changeController(const std::string &ctrl_name);

  void setDataLogging(bool set) { log_data = set; }

  void setReadTargetFromCamera(bool set) { read_target_from_camera = set; }

  std::vector<std::string> controllerTypes() const;

  GmpMpcWin *gui;
  
  // ============  MPC  =============
  
  MpcSettings mpc_cfg;

  void runMPC(bool sim=false);
  void initMPC(gmp_::GMP_MPC::Ptr &gmp_mpc);
  ExecResultMsg loadMpcParams(const std::string &path, MpcSettings &mpc_cfg);
  void viewMpcPath(bool view);
  void plotMpcResults();
  struct{
    arma::rowvec Time;
    arma::mat P_data;
    arma::mat Pdot_data;
    arma::mat Pddot_data;
    arma::mat pos_slack_data;
    arma::mat vel_slack_data;
    arma::mat accel_slack_data;
    arma::mat target_data;
    bool isEmpty() const { return Time.size()==0; }
    void clear() 
    { 
      Time.clear(); P_data.clear(); Pdot_data.clear(); Pddot_data.clear(); 
      pos_slack_data.clear(); vel_slack_data.clear(); accel_slack_data.clear();
      target_data.clear();
    }
    void add(double t, const arma::vec &pos, const arma::vec &vel, const arma::vec &accel, const arma::vec &target,
            const arma::vec &pos_slack={}, const arma::vec &vel_slack={}, const arma::vec &accel_slack={})
    {
      Time = arma::join_horiz( Time, arma::vec({t}) );
      P_data = arma::join_horiz( P_data, pos );
      Pdot_data = arma::join_horiz( Pdot_data, vel );
      Pddot_data = arma::join_horiz( Pddot_data, accel );
      target_data = arma::join_horiz( target_data, target );
      if (!pos_slack.empty()) pos_slack_data = arma::join_horiz( pos_slack_data, pos_slack );
      if (!vel_slack.empty()) vel_slack_data = arma::join_horiz( vel_slack_data, vel_slack );
      if (!accel_slack.empty()) accel_slack_data = arma::join_horiz( accel_slack_data, accel_slack );
    }
    void write(io_::FileIO &fid, const std::string &prefix="")
    {
      fid.write(prefix + "Time", Time);
      fid.write(prefix + "P_data", P_data);
      fid.write(prefix + "Pdot_data", Pdot_data);
      fid.write(prefix + "Pddot_data", Pddot_data);
      fid.write(prefix + "target_data", target_data);
      fid.write(prefix + "pos_slack_data", pos_slack_data);
      fid.write(prefix + "vel_slack_data", vel_slack_data);
      fid.write(prefix + "accel_slack_data", accel_slack_data);
    }

  } mpc_data;

  // ============  Repulsive forces  ============
  void runRepForces(bool sim = false);
  ExecResultMsg loadRepForceParams(const std::string &path);
  void viewRepForcesPath(bool view);
  void plotRepForcesResults();
  struct{
    arma::rowvec Time;
    arma::mat P_data;
    arma::mat Pdot_data;
    arma::mat Pddot_data;
    bool isEmpty() const { return Time.size()==0; }
    void clear() { Time.clear(); P_data.clear(); Pdot_data.clear(); Pddot_data.clear(); }
    void add(double t, const arma::vec &pos, const arma::vec &vel, const arma::vec &accel)
    {
      Time = arma::join_horiz( Time, arma::vec({t}) );
      P_data = arma::join_horiz( P_data, pos );
      Pdot_data = arma::join_horiz( Pdot_data, vel );
      Pddot_data = arma::join_horiz( Pddot_data, accel );
    }
    void write(io_::FileIO &fid, const std::string &prefix="")
    {
      fid.write(prefix + "Time", Time);
      fid.write(prefix + "P_data", P_data);
      fid.write(prefix + "Pdot_data", Pdot_data);
      fid.write(prefix + "Pddot_data", Pddot_data);
    }

  } rep_force_data;

  struct RepForceSettings{
    std::string int_method; // integration method {"ode", "euler"}
    double d0;              // distance after which the repulsive force is activated
    bool pos_on;            // add repulsive force to position
    bool vel_on;            // add repulsive force to velocity
    double kp;              // gain for position repulsive force
    double kv;              // gain for velocity repulsive force
    double K;               // DMP stiffness
    double D;               // DMP damping
  };
  struct RepForceSettings rep_force_cfg;


  // =========== Unconstrained ============
  void runUncostr(bool sim = false);
  void viewUnconstrainedPath(bool view);
  void plotUnconstrainedResults();
  struct{
    arma::rowvec Time;
    arma::mat P_data;
    arma::mat Pdot_data;
    arma::mat Pddot_data;
    bool isEmpty() const { return Time.size()==0; }
    void clear() { Time.clear(); P_data.clear(); Pdot_data.clear(); Pddot_data.clear(); }
    void add(double t, const arma::vec &pos, const arma::vec &vel, const arma::vec &accel)
    {
      Time = arma::join_horiz( Time, arma::vec({t}) );
      P_data = arma::join_horiz( P_data, pos );
      Pdot_data = arma::join_horiz( Pdot_data, vel );
      Pddot_data = arma::join_horiz( Pddot_data, accel );
    }
  } unconstr_data;

  // ========== Offline Optimization ============
  void runOfflineOpt(bool sim = false);
  ExecResultMsg loadOffLineOptParams(const std::string &path);
  int offLineOpt(gmp_::GMP *gmp, const arma::vec &y0, const arma::vec &yg, double tau, std::string *err_msg=0);
  void viewOfflineOptPath(bool view);
  void plotOffLineOptResults();
  struct{
    arma::rowvec Time;
    arma::mat P_data;
    arma::mat Pdot_data;
    arma::mat Pddot_data;
    bool isEmpty() const { return Time.size()==0; }
    void clear() { Time.clear(); P_data.clear(); Pdot_data.clear(); Pddot_data.clear(); }
    void add(double t, const arma::vec &pos, const arma::vec &vel, const arma::vec &accel)
    {
      Time = arma::join_horiz( Time, arma::vec({t}) );
      P_data = arma::join_horiz( P_data, pos );
      Pdot_data = arma::join_horiz( Pdot_data, vel );
      Pddot_data = arma::join_horiz( Pddot_data, accel );
    }
  } offline_opt_data;

  struct{
    double opt_pos_gain;
    double opt_vel_gain;

    double time_limit;
    unsigned max_iter;
    double abs_tol;
    double rel_tol;

    double kernels_trunc_thres;
  } offline_opt_cfg;


  arma::vec getTargetPose(bool *found=0) const
  {
    std::unique_lock<std::mutex> lck(target_mtx);

    if (read_target_from_camera)
    {
      if (!tag_listener) throw std::runtime_error("tag_listener is not initialized...");
      apriltag_ros::AprilTagListener::TagDetection tag = tag_listener->getTagDetection(target_tag_id);
      if (tag.tag_id < 0)
      {
        // failed to detect tag
        // keep the previous pose
      }
      else 
      {
        // update pose
        const_cast<GmpMpcController *>(this)->target_pos = { tag.pos.x(), tag.pos.y(), tag.pos.z() };
        const_cast<GmpMpcController *>(this)->target_quat = { tag.quat.w(), tag.quat.x(), tag.quat.y(), tag.quat.z() };
      }
      if (found) *found = tag.tag_id == target_tag_id;
    }
    else
    {
      if (found) *found = true;
    }
    
    return arma::join_vert(target_pos, target_quat);
  }

  arma::vec getTargetPosition(bool *found=0) const { return getTargetPose(found).subvec(0,2); }

  arma::vec getTargetQuat(bool *found=0) const { return getTargetPose(found).subvec(3,6); }

  apriltag_ros::AprilTagListener *getTagListener() { return tag_listener.get(); }

  rviz_::RvizMarkerPublisher *getRvizPublisher() { return rviz_pub.get(); }

  void sendGuiMsg(const ExecResultMsg &msg)
  {
    emit gui->showMsgSignal(msg);
  }

  void setKinematicLimits(const arma::mat &pos_lim, const arma::mat &vel_lim, const arma::mat &accel_lim)
  {
    this->pos_lim = pos_lim;
    this->vel_lim = vel_lim;
    this->accel_lim = accel_lim;
  }

  void setInitPose(const arma::vec &q0) { this->q_start = q0; }

  bool isOk() const;

  bool runOn() const { return run_on; }

  void setTaskVelocity(const arma::vec &V_cmd, const arma::vec &pos_cmd, const arma::vec &quat_cmd)
  {
    if (use_click) robot->setTaskVelocity(V_cmd, pos_cmd, quat_cmd);
    else robot->setTaskVelocity(V_cmd);
  }
  
  // ======== Experiment ========

  bool initExperiment();
  void startExperiment();
  bool stopExperiment();
  void changeExperiment(const std::string &exp_name);
  std::vector<std::string> experimentTypes() const;

  std::unique_ptr<gmp_mpc_ctrl_::Experiment> exp;
  std::map<std::string, std::function<void()> > resetExpFun_map;

  std::string base_link;
  std::string marker_array_topic;

protected:

  static void printElapsTime(const arma::rowvec &elaps_t_data, const std::string &title="");

  // ============================

  struct ViaPoint
  {
    int tag_id;
    double tx;
    double err_tol;
    arma::vec pos;
  };

  std::list<int> via_point_ids; // ids of unprocessed yet via-points

  // to remove a via-point that has been processed
  void removeViaPoint(int id)
  {
    for (auto it = via_point_ids.begin(); it != via_point_ids.end(); it++)
    {
      if (*it == id)
      {
        via_point_ids.erase(it);
        return;
      }
    }
  }

  std::vector<ViaPoint> getViaPoints()
  {
    if (!tag_listener) throw std::runtime_error("tag_listener is not initialized...");

    std::vector<int> via_point_ids;

    std::vector<apriltag_ros::AprilTagListener::TagDetection> tag_detections = tag_listener->getOperationDetections("via-point");

    for (apriltag_ros::AprilTagListener::TagDetection& tag : tag_detections)
    {

    }

    for (int vp_id : via_point_ids)
    {
      apriltag_ros::AprilTagListener::TagDetection tag = tag_listener->getTagDetection(vp_id);
      if (tag.tag_id < 0)
      {
        // failed to detect tag
        // keep the previous pose
      }
      else 
      {
        // { tag.pos.x(), tag.pos.y(), tag.pos.z() };
        // { tag.quat.w(), tag.quat.x(), tag.quat.y(), tag.quat.z() };
      }
    }
    
  }

  std::map<int, ViaPoint> via_points_config;
  ExecResultMsg loadViaPointsConfig(const std::string &path);

  std::vector<ViaPoint> via_points;

  std::map<std::string, std::function<void()> > ctrl_map;

  void run();
  std::function<void()> run_fun_ptr;

  bool initAprilTagListenerFromFile(const std::string &tag_listener_cfg_file, std::string *err_msg=0);
  void updateCameraPoseInRviz(const arma::mat &Tf_base_camOpt, const std::string &base, const std::string &cam_opt, const std::string &cam_base);

  thr_::Semaphore stop_sem;
  bool run_on;

  bool use_click;

  bool log_data = true;

  std::string default_config_file;
  std::string data_path;
  std::string config_path;
  std::string model_path;

  arma::vec q_start;

  bool read_target_from_camera = false;
  mutable std::mutex target_mtx;
  int target_tag_id = 0;
  arma::vec target_pos;
  arma::vec target_quat;
  double tau;

  arma::mat pos_lim;
  arma::mat vel_lim;
  arma::mat accel_lim;

  bool is_model_loaded = false;
  gmp_::GMP::Ptr gmp;

  std::unique_ptr<rviz_::RvizMarkerPublisher> rviz_pub;

  std::unique_ptr<apriltag_ros::AprilTagListener> tag_listener; 

};

#endif // $_PROJECT_384$_GMP_MPC_CONTROLLER_H

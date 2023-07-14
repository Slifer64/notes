#ifndef $_PROJECT_384$_ROBOT_JOINTS_RECORDER_H
#define $_PROJECT_384$_ROBOT_JOINTS_RECORDER_H

#include <main_controller/utils/timer.h>
#include <robot_wrapper/robot.h>

class RobotJointsRecorder
{
public:
  RobotJointsRecorder(const rw_::Robot *robot);


  void start();
  void stop();

  void saveData(const std::string &path);
  void loadData(const std::string &path);

  arma::rowvec getTime() const { return Time; }
  arma::mat getJointPosData() const { return joint_pos_data; }

  std::string getDefaultPath() const { return default_path; }
  void setDefaultPath(const std::string &path) { default_path = path; }

private:

  void clearData();

  const rw_::Robot *robot_;
  Timer timer;

  arma::rowvec Time;
  arma::mat joint_pos_data;

  bool record_;

  std::string default_path;
};


#endif // $_PROJECT_384$_ROBOT_JOINTS_RECORDER_H
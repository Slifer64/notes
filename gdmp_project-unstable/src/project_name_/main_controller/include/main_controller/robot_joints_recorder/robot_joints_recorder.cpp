#include <main_controller/robot_joints_recorder/robot_joints_recorder.h>

#include <thread>
#include <chrono>

#include <io_lib/file_io.h>

RobotJointsRecorder::RobotJointsRecorder(const rw_::Robot *robot)
{
  robot_ = robot;
}

void RobotJointsRecorder::start()
{
  clearData();

  record_ = true;

  std::thread([this]()
  {
    unsigned long sleep_time = static_cast<unsigned long>(robot_->getCtrlCycle()*1e9);
    timer.start();
    while (record_)
    {
      double t = timer.elapsedSec();
      arma::vec jpos = robot_->getJointsPosition();

      Time = arma::join_horiz(Time, arma::vec({t}));
      joint_pos_data = arma::join_horiz(joint_pos_data, jpos);

      std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
    }

  }).detach();
}

void RobotJointsRecorder::stop()
{
  record_ = false;
}

void RobotJointsRecorder::clearData()
{
  Time.clear();
  joint_pos_data.clear();
}

void RobotJointsRecorder::saveData(const std::string &path)
{
  io_::FileIO fid(path, io_::FileIO::out | io_::FileIO::trunc);
  fid.write("time", Time);
  fid.write("joints", joint_pos_data);
}

void RobotJointsRecorder::loadData(const std::string &path)
{
  io_::FileIO fid(path, io_::FileIO::in);
  fid.read("time", Time);
  fid.read("joints", joint_pos_data);
}

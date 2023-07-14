#ifndef $_PROJECT_384$_MAIN_CONTROLLER_GRIPPER_CTRL_H
#define $_PROJECT_384$_MAIN_CONTROLLER_GRIPPER_CTRL_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <robotiq_85_gripper/robotiq_85_gripper.h>
#include <thread_lib/semaphore.h>
#include <thread_lib/mtx_var.h>

using namespace as64_;

class GripperControl
{
public:

  GripperControl(r85_::R85Gripper *gripper, std::function<bool()> toggle_state=NULL);

  ~GripperControl();

  void setDefaults(unsigned default_open_angle, double default_close_angle, double default_force);

  bool open(unsigned wait_for_ms=0);

  bool close(unsigned wait_for_ms=0);

  bool move(unsigned angle_, unsigned force_, unsigned wait_for_ms=0);

  bool wait(unsigned wait_for_ms);

  void launchControlThread();

private:

  thr_::MtxVar<bool> is_ctrl_running;
  std::thread ctrl_thread;
  r85_::R85Gripper *gripper;
  std::function<bool()> toggle_state;

  thr_::Semaphore sem;
  thr_::MtxVar<bool> move_request;
  thr_::MtxVar<unsigned> angle;
  thr_::MtxVar<unsigned> force;
  thr_::MtxVar<unsigned> open_angle_;
  thr_::MtxVar<unsigned> close_angle_;
  thr_::MtxVar<unsigned> force_;
};

#endif // $_PROJECT_384$_MAIN_CONTROLLER_GRIPPER_CTRL_H

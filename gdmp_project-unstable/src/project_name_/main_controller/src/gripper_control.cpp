#include <main_controller/utils/gripper_control.h>

using namespace as64_;

GripperControl::GripperControl(r85_::R85Gripper *gripper, std::function<bool()> toggle_state)
{
  this->gripper = gripper;
  this->toggle_state = toggle_state;
  setDefaults(90, 10, 25);
}

GripperControl::~GripperControl()
{
  is_ctrl_running.set(false);
  if (ctrl_thread.joinable()) ctrl_thread.join();
}

void GripperControl::setDefaults(unsigned default_open_angle, double default_close_angle, double default_force)
{
  open_angle_.set(default_open_angle);
  close_angle_.set(default_close_angle);
  force_.set(default_force);
}

bool GripperControl::open(unsigned wait_for_ms)
{
  return move(open_angle_.get(), force_.get(), wait_for_ms);
}

bool GripperControl::close(unsigned wait_for_ms)
{
  return move(close_angle_.get(), force_.get(), wait_for_ms);
}

bool GripperControl::move(unsigned angle_, unsigned force_, unsigned wait_for_ms)
{
  angle.set(angle_);
  force.set(force_);
  move_request.set(true);
  if (wait_for_ms) return wait(wait_for_ms);
  return true;
}

bool GripperControl::wait(unsigned wait_for_ms)
{
  return sem.wait_for(wait_for_ms);
}

void GripperControl::launchControlThread()
{ 
  if (is_ctrl_running()) return;
  is_ctrl_running.set(true);
  ctrl_thread = std::thread([this]()
  {
    bool close_gripper_ = false; // assume gripper is initially open

    while (is_ctrl_running())
    {
      if (toggle_state && toggle_state()) // command sent
      {
        close_gripper_ = !close_gripper_; // toggle state
        if (close_gripper_) gripper->move(close_angle_.get(), force_.get());
        else gripper->move(open_angle_.get(), force_.get());
      }

      if (move_request())
      {
        move_request.set(false);
        gripper->move(angle.get(), force.get());
        sem.notify();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  });
}

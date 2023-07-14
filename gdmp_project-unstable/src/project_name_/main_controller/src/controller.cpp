#include <main_controller/controller.h>
#include <main_controller/main_controller.h>

Controller::Controller(MainController *main_ctrl_, const std::string &ctrl_name_)
  : ctrl_name(ctrl_name_), main_ctrl(main_ctrl_), robot(main_ctrl_->robot.get()) {}

void Controller::setMode(rw_::Mode mode) 
{ 
  main_ctrl->setMode(mode); 
}
#ifndef $_PROJECT_384$_ABSTRACT_CONTROLLER_H
#define $_PROJECT_384$_ABSTRACT_CONTROLLER_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <QPushButton>
#include <QWidget>

#include <robot_wrapper/robot.h>

using namespace as64_;

class MainController; // forward declaration
class MainWindow; // forward declaration

class Controller
{
public:
  Controller(MainController *main_ctrl_, const std::string &ctrl_name_);

  virtual QPushButton *createGui(MainWindow *parent) = 0;
//  {
//    QPushbutton *btn = new QPushButton(this->ctrl_name);
//    gui = new ControllerWin(parent);
//    Qbject::connect( btn, &QPushButton::presed, parent, [this](){ gui->launch(); });
//    return btn;
//  }

  void setMode(rw_::Mode mode);

  MainController *main_ctrl;
  rw_::Robot *robot;
  std::string ctrl_name;

protected:

  

};

#endif // $_PROJECT_384$_ABSTRACT_CONTROLLER_H

#ifndef $_PROJECT_384$_GUI_ROBOT_JOINTS_RECORD_DIALOG_H
#define $_PROJECT_384$_GUI_ROBOT_JOINTS_RECORD_DIALOG_H

#include <QDialog>
#include <QMenuBar>
#include <QAction>
#include <QLabel>
#include <QFileDialog>
#include <QLineEdit>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QPushButton>
#include <QMainWindow>

#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>

#include <main_controller/utils.h>
#include <main_controller/robot_joints_recorder/robot_joints_recorder.h>

class MainController; // forward declaration

class RobotJointsRecordWin : public QMainWindow
{
  Q_OBJECT

public:
  RobotJointsRecordWin(RobotJointsRecorder *joints_recorder, QWidget *parent=0);
  ~RobotJointsRecordWin();

public slots:


private:

  RobotJointsRecorder *joints_recorder_;

  QWidget *central_widget;

  QPushButton *start_btn;
  QPushButton *stop_btn;
  bool is_start_pressed;

};

#endif // $_PROJECT_384$_GUI_ROBOT_JOINTS_RECORD_DIALOG_H

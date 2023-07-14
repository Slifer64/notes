#ifndef GLOBAL_GUI_WINDOW_H
#define GLOBAL_GUI_WINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMenu>
#include <QPushButton>
#include <QLabel>
#include <QAction>
#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPalette>
#include <QColor>
#include <QFileDialog>
#include <QThread>

#include <ros/ros.h>

enum MainCtrlAction
{
  SET_MODE_IDLE = 0,
  SET_MODE_FREEDRIVE = 1,
  TRIGGER_EMERGENCY_STOP = 2,
  GOTO_START_POSE = 3,
  SET_CURRENT_POSE_AS_START = 4,
  SET_CURRENT_POSE_AS_TARGET = 5
};

enum TrainCtrlAction
{
  EXEC_TRAJ = 0,
  EXEC_REVESE_TRAJ = 1,
  START_TRAIN = 2,
  STOP_TRAIN = 3
};

struct ControllerParams
{
  int numOfActions() const { return act_id.size(); }

  void resize(size_t n)
  {
    act_id.resize(n);
    act_name.resize(n);
    btn_label.resize(n);
    stylesheet.resize(n);
  }

  std::vector<int> act_id;
  std::vector<std::string> act_name;
  std::vector<std::string> btn_label;
  std::vector<std::string> stylesheet;

  std::string name;
  std::string publish_topic;
};

class GlobalWindow : public QMainWindow
{
  Q_OBJECT

public:

  explicit GlobalWindow(QWidget *parent = 0);
  ~GlobalWindow();

signals:
  void closeSignal();

private:

  std::vector<ControllerParams> loadParams() const;

  std::vector<ros::Publisher> ros_ctrl_pub;

  QFrame *createControllerFrame(const ControllerParams &ctrl_params, int pub_id);

  QWidget *central_widget;
  QStatusBar *status_bar;

  QFont font1, font2;

  ros::NodeHandle node;
  ros::Publisher ros_main_pub;
  ros::Publisher ros_train_pub;

};

#endif // GLOBAL_GUI_WINDOW_H

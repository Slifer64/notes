#ifndef $_PROJECT_384$_GUI_MAINWINDOW_H
#define $_PROJECT_384$_GUI_MAINWINDOW_H

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
#include <QLineEdit>
#include <QFrame>
#include <QColor>
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QThread>

#include <map>
#include <functional>

#include <gui_lib/view_pose_dialog.h>
#include <gui_lib/view_jpos_dialog.h>
#include <gui_lib/view_wrench_dialog.h>

#include <gui_lib/set_joints_pos_dialog.h>

#include <main_controller/robot_joints_recorder/robot_joints_recorder_win.h>

#include <main_controller/utils.h>

class MainController; // forward declaration

using namespace as64_;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:

  explicit MainWindow(MainController *main_controller, QWidget *parent = 0);
  ~MainWindow();

  void trigger_idleBtnPressed() { emit idle_btn->clicked(); }

  void trigger_freedriveBtnPressed() { emit freedrive_btn->clicked(); }

  void trigger_admittanceBtnPressed() { emit admittance_btn->clicked(); }

  void trigger_emergencyStopBtnPressed() { emit emerg_stop_btn->pressed(); }

  void trigger_gotoStartPoseBtnPressed() { emit goto_start_btn->pressed(); }

  void trigger_setCurrentPoseAsStartBtnPressed() { emit set_current_start_btn->pressed(); }

signals:
  void closeSignal();
  void showMsgSignal(ExecResultMsg msg);
  void reachedPoseSignal(ExecResultMsg msg);
  void finishedMotionReplay(ExecResultMsg msg);
  void modeChangedSignal();
  void emergencyStopSignal();
  void startPoseChangedSignal(const arma::vec &q_start, ExecResultMsg msg);

private slots:
  void emergencyStopSlot();

public:

  static MainWindow *main_win;

  MainController *ctrl_;

  RobotJointsRecordWin *joints_rec_win;
  QAction *joints_rec_win_act;

  QWidget *central_widget;
  QStatusBar *status_bar;

  QLineEdit *robot_mode_le;
  QPushButton *freedrive_btn;
  QPushButton *idle_btn;
  //---------------------
  QPushButton *admittance_btn;
  QFrame *adm_frame;
  QFrame *adm_params_frame;
  std::vector<bool> adm_enabled_dofs;
  std::vector<QCheckBox *> adm_dofs_chkbox;
  // --------------------

  std::vector<QPushButton *> ctrl_btn;
  std::vector<QMainWindow *> ctrl_win;

  QPushButton *emerg_stop_btn;
  QPushButton *enable_robot_btn;
  QPushButton *goto_start_btn;
  QPushButton *goto_start_pose_btn;

  QPushButton *goto_pose_btn;
  QLineEdit *goto_pose_le;

  QPushButton *goto_jpos_btn;
  QLineEdit *goto_jpos_le;

  QPushButton *set_current_start_btn;
  QPushButton *replay_recorded_motion_btn;

  gui_::SetJointsPosDialog *set_start_pose_dialog;

  // -------- File menu ---------
  // -------- Edit menu ---------
  QAction *bias_FTsensor_act;
  QAction *set_start_pose_act;
  // -------- View menu ---------
  QAction *view_wrench_act;
  QAction *view_pose_act;
  QAction *view_joints_act;
  QAction *view_start_pose_act;
  QAction *view_start_joint_pos_act;
  // -------- Utils menu ---------
  QAction *print_current_pose_act;
  QAction *print_joints_pos_act;
  QAction *on_robot_cam_callib_act;

private:

  static std::vector<double> getNumbersFromString(const std::string &str);

  void createMenu();

  void updateGUIonGotoPose(bool set);
  void updateGUIonReplayRecMotion(bool set);

  void createActions();
  QFrame *createModeFrame();
  QFrame *createUtilsFrame();

  void createAdmittanceMode();

  static int showMsg(const ExecResultMsg &msg);
  static int showErrorMsg(const QString &msg);
  static int showWarningMsg(const QString &msg);
  static int showQuestionMsg(const QString &msg);
  static int showInfoMsg(const QString &msg);

  // fonts
  QFont font1;
  QFont font2;
  QFont font3;
  QFont font4;

};

#endif // $_PROJECT_384$_GUI_MAINWINDOW_H

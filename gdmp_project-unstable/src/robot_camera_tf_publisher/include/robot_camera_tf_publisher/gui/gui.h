#ifndef ROBOT_CAMERA_TF_PUBLISHER_GUI_H
#define ROBOT_CAMERA_TF_PUBLISHER_GUI_H

#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMenu>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QAction>
#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QPalette>
#include <QColor>
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QThread>

#include <map>
#include <functional>

class Controller; // forward declaration

enum MSG_TYPE
{
  INFO_MSG,
  WARN_MSG,
  ERR_MSG
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:

  explicit MainWindow(Controller *main_controller, QWidget *parent = 0);
  ~MainWindow();

signals:
  void closeSignal();
  void showMsgSignal(const QString &msg, enum MSG_TYPE msg_type);
  void modeChangedSignal();

public:

  static MainWindow *main_win;

  Controller *main_ctrl;

  QWidget *central_widget;
  QStatusBar *status_bar;

  QLineEdit *robot_mode_le;
  QPushButton *freedrive_btn;
  QPushButton *idle_btn;

  QPushButton *publish_robot_cam_tf_btn;
  QCheckBox *auto_pub_chkbox;
  QCheckBox *viz_published_tf_in_rviz_chkbox;


  QPushButton *move2jointpos_btn;
  QLineEdit *move2jointpos_le;
  std::vector<double> move2_jointpos;

  QAction *print_joints_pos_act;


private:

  void createMenu();
  QFrame *createModeFrame();
  QFrame *createUtilsFrame();

  static int showErrorMsg(const QString &msg);
  static int showWarningMsg(const QString &msg);
  static int showQuestionMsg(const QString &msg);
  static int showInfoMsg(const QString &msg);

  // fonts
  QFont font1;
  QFont font2;

};

#endif // ROBOT_CAMERA_TF_PUBLISHER_GUI_H

#ifndef GUI_ONLINE_ADAPT_WIN_H
#define GUI_ONLINE_ADAPT_WIN_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QAction>
#include <QMenuBar>
#include <QFileDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QMainWindow>
#include <QRadioButton>
#include <QDialogButtonBox>
#include <QButtonGroup>


#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>

// #include <gui_lib/phase_ctrl_slider.h>
#include <gui_lib/view_progress_dialog.h>
// #include <gui_lib/set_pose_dialog.h>
// #include <gui_lib/label_lineEdit.h>

#include <main_controller/main_gui.h>


using namespace as64_;

class OnlineAdaptController; // forward declaration


class OnlineAdaptWin : public QMainWindow
{
  Q_OBJECT

public:
  OnlineAdaptWin(OnlineAdaptController *ctrl, MainWindow *parent);
  ~OnlineAdaptWin();

  bool runInSimulation() const { return sim_chkbox->isChecked(); }

  void triggerPhaseStop() { emit triggerPhaseStopSignal(); }
  void triggerAdaptToRobot() { emit triggerAdaptToRobotSignal(); }
  void triggerStopExec() { emit stopCtrlSignal(ExecResultMsg(ExecResultMsg::INFO, "Execution stopped by ther user.")); }

signals:
  void execStartedSignal();
  void execStopedSignal();
  void stopCtrlSignal(const ExecResultMsg &msg);
  void showMsgSignal(const ExecResultMsg &msg);

  void triggerPhaseStopSignal();
  void triggerAdaptToRobotSignal();

public slots:
    void launch();
    void stop();

private:

  QHBoxLayout *createPubFrameLayout(const std::string &name, std::function<void(bool, unsigned)> callback);

  bool is_ctrl_on;

  bool run = false;

  OnlineAdaptController *ctrl_;
  MainWindow *main_win_;

  QPushButton *reload_apriltag_listener_btn;
  QPushButton *load_params_btn;

  QCheckBox *set_current_pose_as_pick_target_chkbox;

  QPushButton *start_btn;
  void startBtnPressed();
  QPushButton *stop_btn;
  void stopBtnPressed();
  QCheckBox *sim_chkbox;
  
  QCheckBox *retract_from_pick_chkbox;
  QCheckBox *exec_place_chkbox;
  QCheckBox *retract_from_place_chkbox;

  QCheckBox *model_adapt_chkbox;
  QCheckBox *phase_stop_chkbox;
  QCheckBox *reset_model_chkbox;

  QAction *save_exec_data_act;
  QAction *load_exec_data_act;
  QAction *save_model_act;
  
  QAction *view_exec_progress_act;
  gui_::ViewProgressDialog *view_progress_dialog;

  QAction *show_markers_act;
  QAction *show_obstacle_act;
  QAction *clear_exec_markers_act;


  void updateGUIonStartStopBtn();

  void createActions();
  void createMenu();

  // =======   Actions   ======
  QAction *load_params_act;
  // // -----------------------

  QWidget *central_widget;

  void closeEvent(QCloseEvent *event) override;

  // fonts
  QFont font1;
  QFont font2;
  QFont font3;
  QFont font4;
};

#endif // GUI_ONLINE_ADAPT_WIN_H

#ifndef $_PROJECT_384$_GMP_MPC_GUI_H
#define $_PROJECT_384$_GMP_MPC_GUI_H

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

#include <main_controller/utils.h>
#include <main_controller/main_gui.h>

#include <gui_lib/label_lineEdit.h>

using namespace as64_;

class GmpMpcController; // forward declaration

class GmpMpcWin : public QMainWindow
{
  Q_OBJECT

public:

  GmpMpcWin(GmpMpcController *ctrl, MainWindow *parent);
  ~GmpMpcWin();

signals:
  void execFinishSignal(const ExecResultMsg &msg);
  void execStoppedSignal(const ExecResultMsg &msg);

  void experimentFinishSignal(const ExecResultMsg &msg);
  void experimentStoppedSignal(const ExecResultMsg &msg);

  void gotoStartJointsPosFinishedSignal(ExecResultMsg msg);
  void showMsgSignal(const ExecResultMsg &msg);

public slots:
    void launch();
    void stop();

private slots:

private:

  void createActions();
  void createMenu();

  GmpMpcController *ctrl_;
  MainWindow *main_win_;

  // menus
  QMenu *file_menu;
  QMenu *edit_menu;
  QMenu *rviz_menu;
  QMenu *plot_menu;
  QMenu *command_menu;
  QMenu *apriltag_listener_menu;

  // =======   Actions   ======
  // ------- File menu ---------
  QAction *load_model_act;
  QAction *save_model_act;
  QAction *load_params_act;
  QAction *save_log_data_act;
  // ------- Edit menu ---------
  QAction *set_current_pose_as_target_act;
  // ------- Rviz menu ---------
  QAction *view_pos_bounds_act;
  QAction *view_unconstr_path_act;
  QAction *view_mpc_path_act;
  QAction *view_rep_force_path_act;
  QAction *view_offline_opt_path_act;
  QAction *view_target_pose_act;
  QAction *clear_all_rviz_markers_act;
  // ------- Plot menu ---------
  QAction *plot_mcp_results_act;
  QAction *plot_rep_force_results_act;
  QAction *plot_unconstr_results_act;
  QAction *plot_offline_opt_results_act;
  QAction *close_all_plots_act;
  // ------- Apriltag listener menu ---------
  QAction *load_apriltag_listener_act;
  QAction *kill_apriltag_listener_act;
  QAction *publish_tags_act;
  std::vector<QAction *> publish_op_tags_act;
  // ------- Command menu ---------
  QAction *goto_start_joints_pos_act;

  bool is_exec_on;
  QPushButton *start_btn;
  void startBtnPressed();
  QPushButton *stop_btn;
  void stopBtnPressed();
  void updateGUIonStartStopBtn();

  QFrame *run_options_frame;
  QCheckBox *log_data_chkbox;
  QCheckBox *read_target_from_camera_chkbox;

  QComboBox *ctrl_cmbox;
  bool is_controller_assigned = false;

  QFrame *createTestingFrame();
  QFrame *createExperimentsFrame();
  bool is_exp_assigned = false;
  bool is_exp_running = false;
  QComboBox *exp_cmbox;
  QPushButton *start_exp_btn;
  QPushButton *stop_exp_btn;
  void updateGUIonExperimentStartStop();

  // fonts
  QFont font1;
  QFont font2;
  QFont font3;
  QFont font4;

  QWidget *central_widget;
  bool run;

  void closeEvent(QCloseEvent *event) override;

  void updateGuiOnAprilTagListener(bool set);

};

#endif // $_PROJECT_384$_GMP_MPC_GUI_H

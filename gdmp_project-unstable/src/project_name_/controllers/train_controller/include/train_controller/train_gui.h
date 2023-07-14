#ifndef GUI_TRAIN_WIN_H
#define GUI_TRAIN_WIN_H

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

// #include <train_controller/gui_utils/utils.h>

#include <gui_lib/phase_ctrl_slider.h>
#include <gui_lib/view_progress_dialog.h>
#include <gui_lib/set_pose_dialog.h>
#include <gui_lib/label_lineEdit.h>

#include <train_controller/utils/param_slider.h>
#include <train_controller/utils/trim_data_dialog.h>
#include <train_controller/utils/remove_stops_dialog.h>

#include <main_controller/main_gui.h>


using namespace as64_;

class TrainController; // forward declaration

enum PathTeachMethod
{
  PathTeach_FREEDRIVE = 0,
  PathTeach_ADMITTANCE = 1
};

enum VelTeachMethod
{
  VelTeach_SLIDER = 0,
  VelTeach_pHRI = 1
};

enum TrainPhase
{
  Train_IDLE_phase = 0,
  Train_Phase1 = 1,  // kinesthetic guidance
  Train_Phase2 = 2,  // velocity profile teaching
  Train_Phase3 = 3,  // adapt learned trajectory
  Train_Phase4 = 4   // execute learned trajectory
};

class TrainWin : public QMainWindow
{
  Q_OBJECT

public:
  TrainWin(TrainController *ctrl, MainWindow *parent);
  ~TrainWin();

  TrainPhase getTeachingPhase() const { return phase; }
  PathTeachMethod getPathTeachMethod() const { return path_teach_method; }
  VelTeachMethod getVelTeachMethod() const { return vel_teach_method; }
  double getSliderValue() const { return train_slider->getSliderPos(); }
  bool allowPathModifications() const { return modify_path_chkbox->isChecked(); }

  double getPhaseFF() const { return phase_ff_le->text().toDouble(); }
  double getAutoTrimMoveThreshold() const { return 0.001*move_thres_le->text().toDouble(); }


  void trigger_executeTrainTrajectoryAction() { emit exec_train_traj_act->triggered(); }
  void trigger_executeReverseTrainTrajectoryAction() { emit exec_rev_train_traj_act->triggered(); }

  void trigger_startTrainAction() { emit start_btn->pressed(); }
  void trigger_stopTrainAction() { emit stop_btn->pressed(); }

  struct
  {
    double tau;
    bool reverse;
  } phase4;

signals:
  void stopTrainingSignal(const ExecResultMsg &msg);

  void execStartedSignal();
  void execStopedSignal();
  void execDirChangedSignal(int dir);

  void setTargetPoseSignal(const arma::vec &target_pose);

  void ctrl_execModelFinishedSignal(const ExecResultMsg &msg);

public slots:
    void launch();
    void stop();

private slots:
    void phaseChanged();
    void velTeachMethodChanged();

private:

  friend MainWindow;

  void createMenu();

  TrainController *ctrl_;
  MainWindow *main_win_;

  QButtonGroup *phase_rbtn;
  QRadioButton *phase1_rbtn;
  QRadioButton *phase2_rbtn;
  QRadioButton *phase3_rbtn;
  QRadioButton *phase4_rbtn;
  QRadioButton *idle_rbtn;
  std::function<void()> set_idle_phase;

  QComboBox *ph1_meth_cmbox;
  QButtonGroup *vel_teach_method_rbtn;

  QRadioButton *slider_rbtn;
  QRadioButton *phri_rbtn;

  QPushButton *start_btn;
  void startBtnPressed();
  QPushButton *stop_btn;
  void stopBtnPressed();
  QPushButton *train_btn;

  QLineEdit *Nkernels_le;
  QComboBox *train_meth_cmbox;
  QComboBox *model_type_cmbox;

  QFrame *params_frame;

  QFrame *phaseCtrl_params_frame;
  gui_::LabelLineEdit *x_damp_;
  gui_::LabelLineEdit *fv_filt_;
  gui_::LabelLineEdit *fv_scaling_;

  gui_::PhaseCtrlSlider *train_slider;

  QFrame *phase4_frame;

  QFrame *imp_params_frame;
    ParamSlider *damp_pos_slider;
    ParamSlider *stiff_pos_slider;
    ParamSlider *damp_orient_slider;
    ParamSlider *stiff_orient_slider;
    QCheckBox *modify_path_chkbox;

  QCheckBox *logging_chkbox;

  QCheckBox *autotrim_train_data_chkbox;
  QLineEdit *move_thres_le;
  QLabel *move_thres_units_lb;

  QCheckBox *phase_ff_chkbox;
  QLineEdit *phase_ff_le;

  QCheckBox *online_adapt_chkbox;
  QCheckBox *display_ref_frame_chkbox;

  TrimDataDialog *trim_data_dialog;
  RemoveStopsDialog *remove_stops_dialog;
  gui_::SetPoseDialog *set_target_dialog;

  QFrame *createEqulizeVelocityProfileFrame();
  QFrame *eq_vel_prof_frame;
    QPushButton *plot_vel_prof_btn;
    QPushButton *eq_vel_prof_btn;
    QLineEdit *eq_iters_le;
    QLineEdit *nom_vel_le;

  // =======   Actions   ======
  QAction *load_train_data_act;
  QAction *load_model_act;
  QAction *load_params_act;
  QAction *save_train_data_act;
  QAction *save_model_act;
  // -----------------------
  QAction *set_target_pose_act;
  QAction *set_current_pose_as_target_act;
  QAction *set_start_pose_from_train_data_act;
  QAction *set_target_pose_from_model_data_act;
  QAction *trim_data_act;
  QAction *undo_trim_data_act;
  QAction *remove_stops_act;
  QAction *clear_train_data_act;
  // -----------------------
  QAction *plot_train_data_act;
  QAction *plot_demo_sim_data_act;
  QAction *close_all_plots_act;
  QAction *view_learned_path_act;
  QAction *view_target_pose_act;
  // -----------------------
  QAction *exec_train_traj_act;
  QAction *exec_rev_train_traj_act;

  gui_::ViewProgressDialog *view_progress_dialog;
  QAction *view_exec_progress_act;

  QWidget *central_widget;

  bool run;

  bool is_train_on;

  TrainPhase phase;
  PathTeachMethod path_teach_method;
  VelTeachMethod vel_teach_method;

  void updateGUIonTrainData();
  void updateGUIonModel(bool set = true);
  void updateGUIonRadioBtn();
  void updateGUIonStartStopBtn();

  void closeEvent(QCloseEvent *event) override;

  void createActions();
  QFrame *createPhaseCtrlFrame();
  QFrame *createImpedanceParamsFrame();
  QFrame *createParamsFrame();
  QFrame *createTrainFrame();

  ExecResultMsg loadParams(const std::string &path);

  // fonts
  QFont font1;
  QFont font2;
  QFont font3;
  QFont font4;

};

#endif // GUI_TRAIN_WIN_H

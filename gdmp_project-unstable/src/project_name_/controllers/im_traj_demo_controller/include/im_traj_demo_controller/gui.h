#ifndef $_PROJECT_384$_IM_TRAJ_DEMO_CONTROLLER_GUI_H
#define $_PROJECT_384$_IM_TRAJ_DEMO_CONTROLLER_GUI_H

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
// #include <gui_lib/view_progress_dialog.h>
// #include <gui_lib/set_pose_dialog.h>
#include <gui_lib/label_lineEdit.h>

#include <main_controller/main_gui.h>

using namespace as64_;

class ImTrajDemoController; // forward declaration

class ImTragDemoGui : public QMainWindow
{
  Q_OBJECT

public:
  ImTragDemoGui(ImTrajDemoController *ctrl, MainWindow *parent);
  ~ImTragDemoGui();

  // void trigger_executeTrainTrajectoryAction() { emit exec_train_traj_act->triggered(); }
  // void trigger_executeReverseTrainTrajectoryAction() { emit exec_rev_train_traj_act->triggered(); }

  void trigger_startTrainAction() { emit start_btn->pressed(); }
  void trigger_stopTrainAction() { emit stop_btn->pressed(); }

  int getSaveCounter() const { return save_counter_le->text().toInt(); }
  void setSaveCounter(int c) { emit setSaveCounterSignal(c); }

  void setImageCount(int c) { emit setImageCountSignal(c); }
signals:

  void setSaveCounterSignal(int c);
  void setImageCountSignal(int c);

  void stopTrainingSignal(const ExecResultMsg &msg);

  void abortMultiImgCaptSignal(const char *msg);

  void execStartedSignal();
  void execStopedSignal();

  void ctrl_execModelFinishedSignal(const ExecResultMsg &msg);

public slots:
    void launch();
    void stop();

private:

  friend MainWindow;

  void createMenu();

  ImTrajDemoController *ctrl_;
  MainWindow *main_win_;

  QPushButton *start_btn;
  void startBtnPressed();
  QPushButton *stop_btn;
  void stopBtnPressed();
  QPushButton *train_btn;

  QPushButton *capt_im_btn;
  QPushButton *capt_multi_im_btn;
  QPushButton *clear_images_btn;
  QLineEdit *img_count_le;
  QPushButton *save_demo_btn;

  QLineEdit *save_counter_le;

  // =======   Actions   ======
  QAction *load_params_act;
  QAction *save_data_act;
  // -----------------------
  QAction *view_learned_path_act;
  // -----------------------
  // QAction *exec_train_traj_act;
  // QAction *exec_rev_train_traj_act;

  QWidget *central_widget;

  bool run;

  bool is_train_on;

  void updateGUIonStartStopBtn();
  void updateGUIonData();

  void closeEvent(QCloseEvent *event) override;

  void createActions();
  QFrame *createTrainFrame();

  // fonts
  QFont font1;
  QFont font2;
  QFont font3;
  QFont font4;

};

#endif // $_PROJECT_384$_IM_TRAJ_DEMO_CONTROLLER_GUI_H

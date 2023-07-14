#ifndef GUI_ADMITTANCE_WIN_H
#define GUI_ADMITTANCE_WIN_H

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

#include <gui_lib/phase_ctrl_slider.h>
#include <gui_lib/view_progress_dialog.h>
#include <gui_lib/set_pose_dialog.h>
#include <gui_lib/label_lineEdit.h>

#include <main_controller/main_gui.h>


using namespace as64_;

class AdmittanceController; // forward declaration


class AdmittanceWin : public QMainWindow
{
  Q_OBJECT

public:
  AdmittanceWin(AdmittanceController *ctrl, MainWindow *parent);
  ~AdmittanceWin();


signals:
  void execStartedSignal();
  void execStopedSignal();
  void stopCtrlSignal(const ExecResultMsg &msg);

public slots:
    void launch();
    void stop();

private:

  bool is_ctrl_on;

  bool run = false;

  AdmittanceController *ctrl_;
  MainWindow *main_win_;

  std::vector<bool> adm_enabled_dofs;
  std::vector<QCheckBox *> adm_dofs_chkbox;

  QComboBox *ctrl_type_cmbox;

  QPushButton *start_btn;
  void startBtnPressed();
  QPushButton *stop_btn;
  void stopBtnPressed();

  void updateGUIonStartStopBtn();

  void createActions();
  void createMenu();


  // =======   Actions   ======
  QAction *load_params_act;
  // // -----------------------
  QAction *plot_train_data_act;
  QAction *close_all_plots_act;
  // // -----------------------

  QWidget *central_widget;

  void closeEvent(QCloseEvent *event) override;

  // fonts
  QFont font1;
  QFont font2;
  QFont font3;
  QFont font4;

  QPushButton *load_params_btn;

};

#endif // GUI_ADMITTANCE_WIN_H

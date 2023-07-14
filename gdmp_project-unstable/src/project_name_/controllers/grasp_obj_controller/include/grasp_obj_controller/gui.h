#ifndef $_PROJECT_384$_GRASP_OBJ_CONTROLLER_GUI_H
#define $_PROJECT_384$_GRASP_OBJ_CONTROLLER_GUI_H

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
#include <map>
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

class GraspObjController; // forward declaration

#include <QTextEdit>
class TextBox : public QTextEdit 
{
public:

    unsigned min_height = 50;

    explicit TextBox(unsigned width=200, bool read_only=true, QWidget* parent = nullptr) : QTextEdit(parent) 
    {
      setReadOnly(read_only);
      setStyleSheet("QTextEdit {background-color: black; color: white;}");
      setFixedWidth(width);
      setMinimumHeight(min_height);

      // QObject::connect(this, &TextBox::setTextSignal, this, [this](const QString& text, QColor color){ this->setText(text, color); });
    }

    void setText(const QString& text, QColor color=QColor(Qt::white)) 
    {
      QTextEdit::setText(text);
      setStyleSheet(QString("QTextEdit {background-color: black; color: %1;}").arg(color.name()));
      document()->adjustSize();
      setFixedHeight(std::max(int(document()->size().height() + 5), minimumHeight()));
    }

// signals:

//   void setTextSignal(const QString& text, QColor color);
};


class GraspObjGui : public QMainWindow
{
  Q_OBJECT

public:
  GraspObjGui(GraspObjController *ctrl, MainWindow *parent);
  ~GraspObjGui();

  std::string getModelName() const { return model_cmbox->currentText().toStdString(); }
  unsigned getTargetObjId() const { return target_obj_map.find(target_obj_cmbox->currentText().toStdString())->second; }

  bool auto_record() const { return autorecord_chkbox->isChecked(); }

  void trigger_startExecAction() { emit start_btn->pressed(); }
  
signals:

  void stopExecSignal(const ExecResultMsg &msg);

  void showMsgSignal(const ExecResultMsg &msg);

  void execStartedSignal();
  void execStopedSignal();

  void ctrl_execModelFinishedSignal(const ExecResultMsg &msg);

public slots:
    void launch();
    void stop();

private:

  friend MainWindow;

  void showMsg(const ExecResultMsg &msg);

  void createMenu();

  void loadGUIParams();

  GraspObjController *ctrl_;
  MainWindow *main_win_;

  QPushButton *start_btn;
  void startBtnPressed();
  QPushButton *stop_btn;
  void stopBtnPressed();
  QPushButton *move_to_start_btn;
  QPushButton *read_img_btn;
  QPushButton *retract_btn;

  QCheckBox *autorecord_chkbox;
  QPushButton *record_btn;

  TextBox *msg_box;

  std::vector<std::string> model_names;
  QComboBox *model_cmbox;

  std::map<std::string, int> target_obj_map;
  QComboBox *target_obj_cmbox;

  // =======   Actions   ======
  QAction *load_params_act;
  // -----------------------
  QAction *view_exec_path_act;
  // -----------------------

  QWidget *central_widget;

  bool run;

  bool is_exec_on;

  void updateGUIonStartStopBtn();

  void closeEvent(QCloseEvent *event) override;

  void createActions();
  QFrame *createStartStopFrame();

  // fonts
  QFont font1;
  QFont font2;
  QFont font3;
  QFont font4;

};

#endif // $_PROJECT_384$_GRASP_OBJ_CONTROLLER_GUI_H

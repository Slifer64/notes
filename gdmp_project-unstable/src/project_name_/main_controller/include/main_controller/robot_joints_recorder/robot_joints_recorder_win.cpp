#include <main_controller/robot_joints_recorder/robot_joints_recorder_win.h>

RobotJointsRecordWin::RobotJointsRecordWin(RobotJointsRecorder *joints_recorder, QWidget *parent) : QMainWindow(parent)
{
  joints_recorder_ = joints_recorder;

  this->resize(200,100);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":Robot Joints recorder");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  // status_bar = new QStatusBar(this);
  // this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  QFont font1 = QFont("Ubuntu", 15, QFont::DemiBold);

  // ==========  Create Widgets  =========
  start_btn = new QPushButton("start");
  start_btn->setFont(font1);
  is_start_pressed = false;
  QObject::connect( start_btn, &QPushButton::pressed, this, [this]()
  {
    if (is_start_pressed) return;
    is_start_pressed = true;
    stop_btn->setEnabled(true);
    start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
    joints_recorder_->start();
  });

  stop_btn = new QPushButton("stop");
  stop_btn->setFont(font1);
  stop_btn->setEnabled(false);
  QObject::connect( stop_btn, &QPushButton::pressed, this, [this]()
  {
    is_start_pressed = false;
    start_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    joints_recorder_->stop();
  });

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addWidget(start_btn);
  main_layout->addWidget(stop_btn);


  // ==========  Create Menu  =========
  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QAction *save_act = new QAction("save", this);
  save_act->setStatusTip("Saves the recorded data.");
  QObject::connect( save_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getSaveFileName(this, tr("Save Robot joints"), joints_recorder_->getDefaultPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    try {
      joints_recorder_->saveData(path);
      showInfoMsg("Saved robot joints!");
    }
    catch (std::exception &e)
    { showErrorMsg(e.what()); }
  });

  QAction *load_act = new QAction("load", this);
  load_act->setStatusTip("Loads the recorded data.");
  QObject::connect( load_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load Robot joints"), joints_recorder_->getDefaultPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    try {
      joints_recorder_->loadData(path);
      showInfoMsg("Loaded robot joints!");
    }
    catch (std::exception &e)
    { showErrorMsg(e.what()); }
  });

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(save_act);
  file_menu->addAction(load_act);

}

RobotJointsRecordWin::~RobotJointsRecordWin()
{

}

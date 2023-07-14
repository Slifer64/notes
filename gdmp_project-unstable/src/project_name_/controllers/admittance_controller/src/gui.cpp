#include <admittance_controller/gui.h>
#include <admittance_controller/controller.h>

#include <QDebug>
#include <iostream>
#include <io_lib/xml_parser.h>
#include <plot_lib/qt_plot.h>

#include <ros/package.h>

AdmittanceWin::AdmittanceWin(AdmittanceController *ctrl, MainWindow *parent): QMainWindow(parent)
{
  this->main_win_ = parent;
  this->ctrl_ = ctrl;

  this->ctrl_->gui = this;

  this->resize(200,200);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":Variable Admittance");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  // status_bar = new QStatusBar(this);
  // this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  run = false;
  is_ctrl_on = false;
  
  // =============  Fonts  ================

  font1 = QFont("Ubuntu", 17, QFont::DemiBold);
  font2 = QFont("Ubuntu", 15, QFont::DemiBold);
  font3 = QFont("Ubuntu", 14, 57);
  font4 = QFont("Ubuntu", 12, QFont::Normal);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // =========  Ctrl type ==========
  QLabel *ctrl_type_lb = new QLabel("ctrl method:");
  ctrl_type_lb->setFont(font2);
  ctrl_type_lb->setAlignment(Qt::AlignCenter);

  ctrl_type_cmbox = new QComboBox();
  ctrl_type_cmbox->setFont(font2);
  ctrl_type_cmbox->addItem("JOINT_POS");
  ctrl_type_cmbox->addItem("JOINT_VEL");
  ctrl_type_cmbox->addItem("CART_VEL");
  QObject::connect( ctrl_type_cmbox, &QComboBox::currentTextChanged, this, [this]()
  { 
    ExecResultMsg msg = ctrl_->setCtrlMethod(ctrl_type_cmbox->currentText().toStdString());
    if (msg.getType()!=ExecResultMsg::INFO) showMsg(msg);
  });
  ctrl_type_cmbox->setCurrentText("JOINT_VEL"); // this should automatically emit the signal below, but for some reason it doesn't...
  // emit ctrl_type_cmbox->currentTextChanged("");

  QHBoxLayout *ctrl_type_layout = new QHBoxLayout;
  ctrl_type_layout->addWidget(ctrl_type_lb);
  ctrl_type_layout->addWidget(ctrl_type_cmbox);
  ctrl_type_layout->addStretch(0);

  // =========  Start - Stop =============
  start_btn = new QPushButton("start");
  start_btn->setFont(font1);
  QObject::connect( start_btn, &QPushButton::pressed, this, [this](){ startBtnPressed(); }) ;

  stop_btn = new QPushButton("stop");
  stop_btn->setFont(font1);
  stop_btn->setEnabled(false);
  QObject::connect( stop_btn, &QPushButton::pressed, this, [this](){ stopBtnPressed(); }) ;
  QObject::connect( this, &AdmittanceWin::stopCtrlSignal, this, [this](const ExecResultMsg &msg)
  {
    is_ctrl_on = false;
    updateGUIonStartStopBtn();
    showMsg(msg);
  });


  QHBoxLayout *start_stop_btns_layout = new QHBoxLayout;
  start_stop_btns_layout->addWidget(start_btn);
  start_stop_btns_layout->addWidget(stop_btn);
  //sst_btns_layout->insertSpacing(2,40);
  start_stop_btns_layout->addStretch(0);

  // =========== Load params =============
  load_params_btn = new QPushButton("load params");
  load_params_btn->setFont(font1);
  QObject::connect( load_params_btn, &QPushButton::pressed, this, [this](){ showMsg(ctrl_->loadParams()); });

  // ============ Layout 1 =============
  QVBoxLayout *layout1 = new QVBoxLayout;
    layout1->addLayout(ctrl_type_layout);
    layout1->addLayout(start_stop_btns_layout);
    layout1->addWidget(load_params_btn);
    layout1->addStretch(0);

  QFrame *frame1 = new QFrame(central_widget);
    frame1->setFrameStyle(QFrame::Box | QFrame::Raised);
    frame1->setLineWidth(4);
    frame1->setLayout(layout1);

  // ============ Main Layout ==============
  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addWidget(frame1);
  main_layout->addStretch(0);
   
  createActions();
  createMenu();
}

void AdmittanceWin::createActions()
{
  // load_params_act = new QAction("load params", this);
  // load_params_act->setStatusTip("Load params from disk.");
  // QObject::connect( load_params_act, &QAction::triggered, this, [this]()
  // {
  //   std::string path = QFileDialog::getOpenFileName(this, tr("Load params"), ctrl_->getDefaultPath().c_str(), "Yaml files (*.yaml)").toStdString();
  //   if (path.empty()) return;
  //   showMsg(this->loadParams(path));
  // });

  // plot_train_data_act = new QAction("demo data", this);
  // plot_train_data_act->setStatusTip("Plots the training data.");
  // QObject::connect( plot_train_data_act, &QAction::triggered, this, [this](){ std::thread([this](){ ctrl_->plotTrainData(); } ).detach(); });

  // close_all_plots_act = new QAction("close all", this);
  // close_all_plots_act->setStatusTip("Closes all figures.");
  // QObject::connect( close_all_plots_act, &QAction::triggered, this, [this]()
  // { std::thread([this](){ pl_::closeAll(); } ).detach(); });

}

void AdmittanceWin::createMenu()
{
  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  // QMenu *file_menu = menu_bar->addMenu(tr("&File"));
  //   file_menu->addAction(load_train_data_act);
  //   file_menu->addAction(load_model_act);
  //   file_menu->addAction(load_params_act);
  //   file_menu->addSeparator();
  //   file_menu->addAction(save_train_data_act);
  //   file_menu->addAction(save_model_act);

  // QMenu *edit_menu = menu_bar->addMenu(tr("&Edit"));
  //   edit_menu->addAction(set_target_pose_act);
  //   edit_menu->addAction(set_current_pose_as_target_act);
  //   edit_menu->addAction(set_start_pose_from_train_data_act);
  //   edit_menu->addAction(set_target_pose_from_model_data_act);
  //   edit_menu->addSeparator();
  //   edit_menu->addAction(trim_data_act);
  //   edit_menu->addAction(undo_trim_data_act);
  //   edit_menu->addAction(remove_stops_act);
  //   edit_menu->addAction(clear_train_data_act);

  // QMenu *view_menu = menu_bar->addMenu(tr("&View"));
  //   QMenu *plot_menu = view_menu->addMenu(tr("&Plot"));
  //     plot_menu->addAction(plot_train_data_act);
  //     plot_menu->addAction(plot_demo_sim_data_act);
  //     plot_menu->addAction(close_all_plots_act);
  //   QMenu *rviz_menu = view_menu->addMenu(tr("&Rviz"));
  //     rviz_menu->addAction(view_learned_path_act);
  //     rviz_menu->addAction(view_target_pose_act);
  //   view_menu->addAction(view_exec_progress_act);

  // QMenu *cmd_menu = menu_bar->addMenu(tr("&Command"));
  //   cmd_menu->addAction(exec_train_traj_act);
  //   cmd_menu->addAction(exec_rev_train_traj_act);
}

AdmittanceWin::~AdmittanceWin()
{
    stop();
}

void AdmittanceWin::launch()
{
  if (!run)
  {
    run = true;
    this->show();
  }
}


void AdmittanceWin::stop()
{
  if (run) run = false;
  this->hide();
}

void AdmittanceWin::startBtnPressed()
{
  if (is_ctrl_on) return;
  is_ctrl_on = true;
  updateGUIonStartStopBtn();
  emit execStartedSignal();
  std::thread([this](){ ctrl_->start(); }).detach();
}

void AdmittanceWin::stopBtnPressed()
{
  if (!is_ctrl_on) return;
  is_ctrl_on = false;
  updateGUIonStartStopBtn();
  emit execStopedSignal();
  showMsg(ctrl_->stop());
}

void AdmittanceWin::updateGUIonStartStopBtn()
{
  if (is_ctrl_on) start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  else start_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  stop_btn->setEnabled(is_ctrl_on);
  ctrl_type_cmbox->setEnabled(!is_ctrl_on);
}

void AdmittanceWin::closeEvent(QCloseEvent *event)
{
  stop();
}


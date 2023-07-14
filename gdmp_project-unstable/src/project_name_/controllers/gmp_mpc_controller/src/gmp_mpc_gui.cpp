#include <gmp_mpc_controller/gmp_mpc_gui.h>
#include <gmp_mpc_controller/gmp_mpc_controller.h>

#include <main_controller/main_gui.h>

#include <QDebug>
#include <iostream>
#include <io_lib/xml_parser.h>
#include <plot_lib/qt_plot.h>

#include <ros/package.h>

GmpMpcWin::GmpMpcWin(GmpMpcController *ctrl, MainWindow *parent): QMainWindow(parent)
{
  this->main_win_ = parent;
  this->ctrl_ = ctrl;

  this->ctrl_->gui = this;

  this->resize(400,200);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":GMP - MPC");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  // status_bar = new QStatusBar(this);
  // this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  run = false;

  // =============  Fonts  ================

  font1 = QFont("Ubuntu", 17, QFont::DemiBold);
  font2 = QFont("Ubuntu", 15, QFont::DemiBold);
  font3 = QFont("Ubuntu", 14, 57);
  font4 = QFont("Ubuntu", 12, QFont::Normal);

  QObject::connect(this, &GmpMpcWin::showMsgSignal, this, [this](const ExecResultMsg &msg) { showMsg(msg); } );

  createActions();
  createMenu();

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  {
    QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
    main_layout->addWidget(createTestingFrame());
    main_layout->addWidget(createExperimentsFrame());
    main_layout->addStretch(0);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  // to initialize
  updateGuiOnAprilTagListener(false); 
  updateGUIonStartStopBtn();
  updateGUIonExperimentStartStop();
}

GmpMpcWin::~GmpMpcWin()
{
    stop();
}

QFrame *GmpMpcWin::createTestingFrame()
{
  QLabel *title = new QLabel("Testing");
  title->setFont(font1);
  title->setAlignment(Qt::AlignCenter);
  title->setStyleSheet("color:rgb(160,100,0); background-color:rgba(210, 210, 210, 100);");

  QLabel *ctrl_label = new QLabel("Controller:");
  ctrl_label->setFont(font2);

  ctrl_cmbox = new QComboBox;
  ctrl_cmbox->setFont(font2);
  ctrl_cmbox->addItem(""); // add a dummy
  for ( const std::string &ctrl_name : ctrl_->controllerTypes() ) ctrl_cmbox->addItem(ctrl_name.c_str());
  QObject::connect( ctrl_cmbox, &QComboBox::currentTextChanged, this, [this](const QString &ctrl_name)
  { 
    is_controller_assigned = !ctrl_name.isEmpty();
    if (is_controller_assigned) ctrl_->changeController(ctrl_name.toStdString());
    start_btn->setEnabled(ctrl_->isModelLoaded() && is_controller_assigned);
  });
  ctrl_cmbox->setCurrentText("");

  QHBoxLayout *ctrl_cmbox_layout = new QHBoxLayout;
  ctrl_cmbox_layout->addWidget(ctrl_label);
  ctrl_cmbox_layout->addWidget(ctrl_cmbox);
  ctrl_cmbox_layout->addStretch(0);

  // =================================================

  is_exec_on = false;
  start_btn = new QPushButton("start");
  start_btn->setFont(font1);
  start_btn->setEnabled(false);
  QObject::connect( start_btn, &QPushButton::pressed, this, [this]()
  {
    if (is_exec_on) return;
    is_exec_on = true;
    updateGUIonStartStopBtn();
    ctrl_->startExec();
  });
  QObject::connect(this, &GmpMpcWin::execStoppedSignal, this, [this](const ExecResultMsg &msg)
  {
    is_exec_on = false;
    updateGUIonStartStopBtn();
    if (!msg.empty()) showMsg(msg);
  });
  QObject::connect(this, &GmpMpcWin::execFinishSignal, this, [this](const ExecResultMsg &msg)
  {
    is_exec_on = false;
    updateGUIonStartStopBtn();
    if (!msg.empty()) showMsg(msg);
  });

  stop_btn = new QPushButton("stop");
  stop_btn->setFont(font1);
  stop_btn->setEnabled(false);
  QObject::connect( stop_btn, &QPushButton::pressed, this, [this]()
  {
    if (!is_exec_on) return;
    if (!ctrl_->stopExec()) showErrorMsg("Failed to stop the controller...");
    is_exec_on = false;
    updateGUIonStartStopBtn();
  });

  // =========== Run options =============
  log_data_chkbox = new QCheckBox("log data");
  log_data_chkbox->setFont(font2);
  log_data_chkbox->setChecked(true);
  QObject::connect(log_data_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->setDataLogging(log_data_chkbox->isChecked()); });

  read_target_from_camera_chkbox = new QCheckBox("read target from camera");
  read_target_from_camera_chkbox->setFont(font2);
  read_target_from_camera_chkbox->setChecked(false);
  read_target_from_camera_chkbox->setEnabled(false);
  QObject::connect(read_target_from_camera_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->setReadTargetFromCamera(read_target_from_camera_chkbox->isChecked()); });

  QVBoxLayout *run_options_layout = new QVBoxLayout;
  run_options_layout->addWidget(log_data_chkbox);
  run_options_layout->addWidget(read_target_from_camera_chkbox);

  run_options_frame = new QFrame;
  run_options_frame->setLayout(run_options_layout);

  // -------------------------------------

  QVBoxLayout *exec_layout = new QVBoxLayout;
    exec_layout->addWidget(title);
    exec_layout->addLayout(ctrl_cmbox_layout);
    exec_layout->addWidget(run_options_frame);
    QHBoxLayout *btns_layout = new QHBoxLayout;
      btns_layout->addWidget(start_btn);
      btns_layout->addWidget(stop_btn);
    exec_layout->addLayout(btns_layout);
    exec_layout->addStretch(0);

  QFrame *frame = new QFrame;
  frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  frame->setLineWidth(1);
  frame->setLayout(exec_layout);

  return frame;
}

QFrame *GmpMpcWin::createExperimentsFrame()
{
  QLabel *title = new QLabel("Experiments");
  title->setFont(font1);
  title->setAlignment(Qt::AlignCenter);
  title->setStyleSheet("color:rgb(0,200,0); background-color:rgba(210, 210, 210, 100);");

  QLabel *exp_label = new QLabel("Choose:");
  exp_label->setFont(font2);

  exp_cmbox = new QComboBox;
  exp_cmbox->setFont(font2);
  exp_cmbox->addItem(""); // add a dummy
  for ( const std::string &exp_name : ctrl_->experimentTypes() ) exp_cmbox->addItem(exp_name.c_str());
  QObject::connect( exp_cmbox, &QComboBox::currentTextChanged, this, [this](const QString &exp_name)
  { 
    is_exp_assigned = !exp_name.isEmpty();
    if (is_exp_assigned) ctrl_->changeExperiment(exp_name.toStdString());
    start_exp_btn->setEnabled( ctrl_->initExperiment() );
  });
  exp_cmbox->setCurrentText("");

  QHBoxLayout *exp_cmbox_layout = new QHBoxLayout;
  exp_cmbox_layout->addWidget(exp_label);
  exp_cmbox_layout->addWidget(exp_cmbox);
  exp_cmbox_layout->addStretch(0);

  is_exp_running = false;

  start_exp_btn = new QPushButton("start");
  start_exp_btn->setFont(font1);
  start_exp_btn->setEnabled(false);
  QObject::connect( start_exp_btn, &QPushButton::pressed, this, [this]()
  {
    if (is_exp_running) return;
    is_exp_running = true;
    updateGUIonExperimentStartStop();
    ctrl_->startExperiment();
  });

  stop_exp_btn = new QPushButton("stop");
  stop_exp_btn->setFont(font1);
  stop_exp_btn->setEnabled(false);
  QObject::connect( stop_exp_btn, &QPushButton::pressed, this, [this]()
  {
    if (!is_exp_running) return;
    if (!ctrl_->stopExperiment()) showErrorMsg("Failed to stop the experiment...");
    is_exp_running = false;
    updateGUIonExperimentStartStop();
  });

  QObject::connect(this, &GmpMpcWin::experimentStoppedSignal, this, [this](const ExecResultMsg &msg)
  {
    is_exp_running = false;
    std::cerr << "is_exp_running = " << is_exp_running << "\n";
    updateGUIonExperimentStartStop();
    if (!msg.empty()) showMsg(msg);
  });

  QObject::connect(this, &GmpMpcWin::experimentFinishSignal, this, [this](const ExecResultMsg &msg)
  {
    is_exp_running = false;
    std::cerr << "is_exp_running = " << is_exp_running << "\n";
    updateGUIonExperimentStartStop();
    if (!msg.empty()) showMsg(msg);
  });

  QHBoxLayout *start_stop_layout = new QHBoxLayout;
  start_stop_layout->addWidget(start_exp_btn);
  start_stop_layout->addWidget(stop_exp_btn);
  start_stop_layout->addStretch(0);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(title);
  layout->addLayout(exp_cmbox_layout);
  layout->addLayout(start_stop_layout);
  layout->addStretch(0);
  
  QFrame *frame = new QFrame();
  frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  frame->setLineWidth(1);
  frame->setLayout(layout);

  return frame;
}

void GmpMpcWin::updateGUIonExperimentStartStop()
{
  if (is_exp_running) start_exp_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  else start_exp_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  stop_exp_btn->setEnabled(is_exp_running);
  exp_cmbox->setEnabled(!is_exp_running);
}

void GmpMpcWin::createActions()
{
  // ------------- File menu actions --------------
  load_model_act = new QAction("load model", this);
  load_model_act->setStatusTip("Loads the gmp model from the disk.");
  QObject::connect( load_model_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load model"), ctrl_->getModelPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    ExecResultMsg msg = ctrl_->loadModel(path);
    if (msg.getType() == ExecResultMsg::INFO)
    {
      // update gui
      save_model_act->setEnabled(true);
      start_btn->setEnabled(ctrl_->isModelLoaded() && is_controller_assigned);
      view_unconstr_path_act->setEnabled(true);
      view_mpc_path_act->setEnabled(true);
      view_rep_force_path_act->setEnabled(true);
      view_offline_opt_path_act->setEnabled(true);
    }
    showMsg(msg);
  });

  save_model_act = new QAction("Save model", this);
  save_model_act->setStatusTip("Save the gmp model in the disk.");
  save_model_act->setEnabled(false);
  QObject::connect( save_model_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getSaveFileName(this, tr("Save model"), ctrl_->getConfigPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    showMsg(ctrl_->saveModel(path));
  });

  load_params_act = new QAction("load params", this);
  load_params_act->setStatusTip("Loads the parameters.");
  QObject::connect( load_params_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load params"), ctrl_->getConfigPath().c_str(), "Yaml files (*.yaml)").toStdString();
    if (path.empty()) return;
    showMsg(ctrl_->loadParams(path));
  });

  save_log_data_act = new QAction("Save logged data", this);
  save_log_data_act->setStatusTip("Saves the logged data in the disk.");
  // save_log_data_act->setEnabled(false);
  QObject::connect( save_log_data_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getSaveFileName(this, tr("Save logged data"), ctrl_->getDataPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    showMsg(ctrl_->saveLogData(path));
  });

  // ------------- Edit menu actions --------------
  set_current_pose_as_target_act = new QAction("Set current pose as target", this);
  set_current_pose_as_target_act->setStatusTip("Sets the current robot pose as the target pose.");
  QObject::connect( set_current_pose_as_target_act, &QAction::triggered, this, [this]()
  { ctrl_->setCurrentPoseAsTarget(); });

  // ------------- Rviz menu actions --------------

  view_pos_bounds_act = new QAction("pos bounds", this);
  view_pos_bounds_act->setStatusTip("Display the pos bounds in rviz.");
  view_pos_bounds_act->setCheckable(true);
  view_pos_bounds_act->setEnabled(true);
  QObject::connect( view_pos_bounds_act, &QAction::triggered, this, [this]()
  {
    static bool enabled = false;

    enabled = !enabled; 
    view_pos_bounds_act->setChecked(enabled);
    ctrl_->viewPosBounds(enabled);
  });

  view_unconstr_path_act = new QAction("unconstrained path", this);
  view_unconstr_path_act->setStatusTip("Publishes the unconstrained path to rviz.");
  view_unconstr_path_act->setCheckable(true);
  view_unconstr_path_act->setEnabled(ctrl_->isModelLoaded());
  QObject::connect( view_unconstr_path_act, &QAction::triggered, this, [this]()
  {
    static bool enabled = false;

    enabled = !enabled; 
    view_unconstr_path_act->setChecked(enabled);
    ctrl_->viewUnconstrainedPath(enabled);
  });
  
  view_mpc_path_act = new QAction("mpc path", this);
  view_mpc_path_act->setStatusTip("Publishes the constrained mpc path to rviz.");
  view_mpc_path_act->setCheckable(true);
  view_mpc_path_act->setEnabled(ctrl_->isModelLoaded());
  QObject::connect( view_mpc_path_act, &QAction::triggered, this, [this]()
  {
    static bool enabled = false;

    enabled = !enabled; 
    view_mpc_path_act->setChecked(enabled);
    ctrl_->viewMpcPath(enabled);
  });

  view_rep_force_path_act = new QAction("rep-force path", this);
  view_rep_force_path_act->setStatusTip("Publishes the repulsive forces path to rviz.");
  view_rep_force_path_act->setCheckable(true);
  view_rep_force_path_act->setEnabled(ctrl_->isModelLoaded());
  QObject::connect( view_rep_force_path_act, &QAction::triggered, this, [this]()
  {
    static bool enabled = false;

    enabled = !enabled; 
    view_rep_force_path_act->setChecked(enabled);
    ctrl_->viewRepForcesPath(enabled);
  });

  view_offline_opt_path_act = new QAction("offline opt path", this);
  view_offline_opt_path_act->setStatusTip("Publishes the offline optimized path to rviz.");
  view_offline_opt_path_act->setCheckable(true);
  view_offline_opt_path_act->setEnabled(ctrl_->isModelLoaded());
  QObject::connect( view_offline_opt_path_act, &QAction::triggered, this, [this]()
  {
    static bool enabled = false;

    enabled = !enabled; 
    view_offline_opt_path_act->setChecked(enabled);
    ctrl_->viewOfflineOptPath(enabled);
  });

  view_target_pose_act = new QAction("target pose", this);
  view_target_pose_act->setStatusTip("Display the target pose frame in rviz.");
  view_target_pose_act->setCheckable(true);
  QObject::connect( view_target_pose_act, &QAction::triggered, this, [this]()
  {
    static bool enabled = false;
    enabled = !enabled;
    view_target_pose_act->setChecked(enabled);
    ctrl_->viewTargetPose(enabled);
  });

  clear_all_rviz_markers_act = new QAction("clear all", this);
  clear_all_rviz_markers_act->setStatusTip("Clears all markers in rviz.");
  QObject::connect( clear_all_rviz_markers_act, &QAction::triggered, this, [this]()
  {
    if (view_unconstr_path_act->isChecked()) emit view_unconstr_path_act->triggered(false);
    if (view_mpc_path_act->isChecked()) emit view_mpc_path_act->triggered(false);
    if (view_rep_force_path_act->isChecked()) emit view_rep_force_path_act->triggered(false);
    if (view_offline_opt_path_act->isChecked()) emit view_offline_opt_path_act->triggered(false);
    if (view_target_pose_act->isChecked()) emit view_target_pose_act->triggered(false);
    // ctrl_->clearAllMarkers(); 
  });

  // ------------- Plot menu actions --------------

  plot_mcp_results_act = new QAction("mpc results", this);
  plot_mcp_results_act->setStatusTip("Plots the MPC results.");
  QObject::connect( plot_mcp_results_act, &QAction::triggered, this, [this]()
  { std::thread([this](){ ctrl_->plotMpcResults(); } ).detach(); });

  plot_rep_force_results_act = new QAction("rep-force results", this);
  plot_rep_force_results_act->setStatusTip("Plots the rep-force results.");
  QObject::connect( plot_rep_force_results_act, &QAction::triggered, this, [this]()
  { std::thread([this](){ ctrl_->plotRepForcesResults(); } ).detach(); });

  plot_unconstr_results_act = new QAction("unconstr results", this);
  plot_unconstr_results_act->setStatusTip("Plots the unconstrained gmp results.");
  QObject::connect( plot_unconstr_results_act, &QAction::triggered, this, [this]()
  { std::thread([this](){ ctrl_->plotUnconstrainedResults(); } ).detach(); });

  plot_offline_opt_results_act = new QAction("offline-opt results", this);
  plot_offline_opt_results_act->setStatusTip("Plots the offline optimization gmp results.");
  QObject::connect( plot_offline_opt_results_act, &QAction::triggered, this, [this]()
  { std::thread([this](){ ctrl_->plotOffLineOptResults(); } ).detach(); });

  close_all_plots_act = new QAction("close all", this);
  close_all_plots_act->setStatusTip("Closes all figures.");
  QObject::connect( close_all_plots_act, &QAction::triggered, this, [this]()
  { std::thread([this](){ pl_::closeAll(); } ).detach(); });

  // ------------- Command menu actions --------------
  goto_start_joints_pos_act = new QAction("Goto start joints pos", this);
  goto_start_joints_pos_act->setStatusTip("Move the robot to the start joints configuration.");
  QObject::connect( goto_start_joints_pos_act, &QAction::triggered, this, [this]()
  { 
    //this->setEnabled(false); 
    ctrl_->gotoStartPose(); 
  });
  QObject::connect( this, &GmpMpcWin::gotoStartJointsPosFinishedSignal, this, [this](ExecResultMsg msg)
  { 
    //this->setEnabled(true); 
    showMsg(msg); 
  });

  // ------------- Apriltag menu actions --------------

  load_apriltag_listener_act = new QAction("load", this);
  load_apriltag_listener_act->setStatusTip("Loads an apriltag listener, initializing according to the settings in the loaded config file.");
  QObject::connect( load_apriltag_listener_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load apriltag listener setttings"), ctrl_->getConfigPath().c_str(), "Yaml files (*.yaml)").toStdString();
    if (path.empty()) return;
    showMsg(ctrl_->loadAprilTagListener(path));
    updateGuiOnAprilTagListener(true);
  });

  kill_apriltag_listener_act = new QAction("Kil", this);
  kill_apriltag_listener_act->setStatusTip("Kill the loaded apriltag listener.");
  QObject::connect( kill_apriltag_listener_act, &QAction::triggered, this, [this]()
  {
    showMsg(ctrl_->killAprilTagListener());
    updateGuiOnAprilTagListener(false);
  });
  
  publish_tags_act = new QAction("publish tf", this);
  publish_tags_act->setStatusTip("Enable/disable publish of the detected tags to TF.");
  publish_tags_act->setCheckable(true);
  QObject::connect( publish_tags_act, &QAction::triggered, this, [this]()
  {
    static bool set = false;
    set = !set;
    publish_tags_act->setChecked(set);
    ctrl_->publishDetectionsToTf(set);
  });

}

void GmpMpcWin::createMenu()
{
  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(load_model_act);
  file_menu->addAction(save_model_act);
  file_menu->addSeparator();
  file_menu->addAction(load_params_act);
  file_menu->addSeparator();
  file_menu->addAction(save_log_data_act);

  edit_menu = menu_bar->addMenu(tr("&Edit"));
  edit_menu->addAction(set_current_pose_as_target_act);

  rviz_menu = menu_bar->addMenu(tr("&Rviz"));
  rviz_menu->addAction(view_pos_bounds_act);
  rviz_menu->addSeparator();
  rviz_menu->addAction(view_unconstr_path_act);
  rviz_menu->addAction(view_mpc_path_act);
  rviz_menu->addAction(view_rep_force_path_act);
  rviz_menu->addAction(view_offline_opt_path_act);
  rviz_menu->addAction(view_target_pose_act);
  rviz_menu->addSeparator();
  rviz_menu->addAction(clear_all_rviz_markers_act);

  // ------- Plot menu ---------
  plot_menu = menu_bar->addMenu(tr("&Plot"));
  plot_menu->addAction(plot_mcp_results_act);
  plot_menu->addAction(plot_rep_force_results_act);
  plot_menu->addAction(plot_unconstr_results_act);
  plot_menu->addAction(plot_offline_opt_results_act);
  plot_menu->addSeparator();
  plot_menu->addAction(close_all_plots_act);

  command_menu = menu_bar->addMenu(tr("&Command"));
  command_menu->addAction(goto_start_joints_pos_act);
   
  apriltag_listener_menu = menu_bar->addMenu(tr("&Tag listener"));
  apriltag_listener_menu->addAction(load_apriltag_listener_act);
  apriltag_listener_menu->addAction(kill_apriltag_listener_act);
  apriltag_listener_menu->addAction(publish_tags_act);
}

void GmpMpcWin::launch()
{
  if (!run)
  {
    run = true;
    this->show();
  }
}

void GmpMpcWin::stop()
{
  if (run) run = false;
  this->hide();
}

void GmpMpcWin::closeEvent(QCloseEvent *event)
{
  stop();
}

void GmpMpcWin::updateGUIonStartStopBtn()
{
  if (is_exec_on) start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  else start_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  bool is_model_loaded = ctrl_->isModelLoaded();
  bool is_log_data = ctrl_->isLogData();

  run_options_frame->setEnabled(!is_exec_on);
  stop_btn->setEnabled(is_exec_on);
  ctrl_cmbox->setEnabled(!is_exec_on);
  // save_log_data_act->setEnabled(is_log_data);
}

void GmpMpcWin::updateGuiOnAprilTagListener(bool set)
{
  if (set)
  {
    // clear previous actions if they exist
    if (!publish_op_tags_act.empty())
    {
      for (QAction *act : publish_op_tags_act) apriltag_listener_menu->removeAction(act);
      publish_op_tags_act.clear();
    }

    for (const std::string op_name : ctrl_->op_names)
    {
      QAction *act = new QAction(("publish " + op_name + " tags").c_str(), this);
      act->setStatusTip(("Publish detected '" + op_name + "' tags to TF.").c_str());
      act->setCheckable(true);
      QObject::connect( act, &QAction::triggered, this, [this, act, op_name]()
      {
        static bool set = false;
        set = !set;
        act->setChecked(set);
        ctrl_->publishOperationTags(op_name,set);
      });

      publish_op_tags_act.push_back(act);
    }
    for (QAction *act : publish_op_tags_act) apriltag_listener_menu->addAction(act);
  }

  kill_apriltag_listener_act->setEnabled(set);
  publish_tags_act->setEnabled(set);
  read_target_from_camera_chkbox->setEnabled(set);
  for (QAction *act : publish_op_tags_act) act->setEnabled(set);
}
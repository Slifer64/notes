#include <online_adapt_controller/gui.h>
#include <online_adapt_controller/controller.h>

#include <main_controller/utils.h>

#include <QDebug>
#include <iostream>
#include <io_lib/xml_parser.h>
#include <plot_lib/qt_plot.h>

#include <ros/package.h>

OnlineAdaptWin::OnlineAdaptWin(OnlineAdaptController *ctrl, MainWindow *parent): QMainWindow(parent)
{
  this->main_win_ = parent;
  this->ctrl_ = ctrl;

  this->ctrl_->gui = this;

  this->resize(200,200);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":Online Adaptation");

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

  // =========  Start - Stop =============
  start_btn = new QPushButton("start");
  start_btn->setFont(font1);
  QObject::connect( start_btn, &QPushButton::pressed, this, [this](){ startBtnPressed(); }) ;

  stop_btn = new QPushButton("stop");
  stop_btn->setFont(font1);
  stop_btn->setEnabled(false);
  QObject::connect( stop_btn, &QPushButton::pressed, this, [this](){ stopBtnPressed(); }) ;
  QObject::connect( this, &OnlineAdaptWin::stopCtrlSignal, this, [this](const ExecResultMsg &msg)
  {
    if (!is_ctrl_on) return;
    ExecResultMsg msg0 = ctrl_->stop();
    is_ctrl_on = false;
    updateGUIonStartStopBtn();
    if (msg0.getType() == ExecResultMsg::INFO) showMsg(msg);
    else showMsg(ExecResultMsg(msg.getType(), msg.getMsg() + "\n" + msg0.getMsg()));
  });

  QHBoxLayout *start_stop_btns_layout = new QHBoxLayout;
  start_stop_btns_layout->addWidget(start_btn);
  start_stop_btns_layout->addWidget(stop_btn);
  //sst_btns_layout->insertSpacing(2,40);
  start_stop_btns_layout->addStretch(0);

  sim_chkbox = new QCheckBox("simulation");
  sim_chkbox->setFont(font2);
  sim_chkbox->setChecked(true);
  QObject::connect(sim_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->sim = sim_chkbox->isChecked(); });


  // ============== Execution pipeline frame ================
  retract_from_pick_chkbox = new QCheckBox("retract from pick");
  retract_from_pick_chkbox->setFont(font2);
  retract_from_pick_chkbox->setChecked(false);
  QObject::connect(retract_from_pick_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->execute_reverse = retract_from_pick_chkbox->isChecked(); });

  exec_place_chkbox = new QCheckBox("place box");
  exec_place_chkbox->setFont(font2);
  exec_place_chkbox->setChecked(false);
  QObject::connect(exec_place_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->execute_place = exec_place_chkbox->isChecked(); });

  retract_from_place_chkbox = new QCheckBox("retract from place");
  retract_from_place_chkbox->setFont(font2);
  retract_from_place_chkbox->setChecked(false);
  QObject::connect(retract_from_place_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->retract_from_place = retract_from_place_chkbox->isChecked(); });

  QVBoxLayout *exec_pipeline_layout = new QVBoxLayout;
  exec_pipeline_layout->addWidget(retract_from_pick_chkbox);
  exec_pipeline_layout->addWidget(exec_place_chkbox);
  exec_pipeline_layout->addWidget(retract_from_place_chkbox);
  exec_pipeline_layout->addStretch(0);

  QFrame *exec_pipeline_frame = new QFrame;
    exec_pipeline_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
    exec_pipeline_frame->setLineWidth(2);
    exec_pipeline_frame->setLayout(exec_pipeline_layout);

  // ========================================================

  model_adapt_chkbox = new QCheckBox("adapt to robot state");
  model_adapt_chkbox->setFont(font3);
  model_adapt_chkbox->setChecked(false);
  QObject::connect(model_adapt_chkbox, &QCheckBox::stateChanged, this, [this]()
  { 
    if (model_adapt_chkbox->isChecked()) phase_stop_chkbox->setChecked(false);
    ctrl_->enable_model_adapt.set(model_adapt_chkbox->isChecked()); 
  });

  phase_stop_chkbox = new QCheckBox("phase stop");
  phase_stop_chkbox->setFont(font3);
  phase_stop_chkbox->setChecked(false);
  QObject::connect(phase_stop_chkbox, &QCheckBox::stateChanged, this, [this]()
  { 
    if (phase_stop_chkbox->isChecked()) model_adapt_chkbox->setChecked(false);
    ctrl_->phase_stop.set(phase_stop_chkbox->isChecked()); 
  });

  QObject::connect(this, &OnlineAdaptWin::triggerPhaseStopSignal, this, [this]()
  {   
    phase_stop_chkbox->setChecked(true);
    // emit phase_stop_chkbox->stateChanged(true);
  });

  QObject::connect(this, &OnlineAdaptWin::triggerAdaptToRobotSignal, this, [this]()
  {   
    model_adapt_chkbox->setChecked(true);
    // emit model_adapt_chkbox->stateChanged(true);
  });

  reset_model_chkbox = new QCheckBox("reset pick model");
  reset_model_chkbox->setFont(font3);
  reset_model_chkbox->setChecked(true);
  QObject::connect(reset_model_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->reset_model.set(reset_model_chkbox->isChecked()); });
  emit reset_model_chkbox->stateChanged(0);

  // ============ Layout 1 =============
  QVBoxLayout *layout1 = new QVBoxLayout;
    layout1->addLayout(start_stop_btns_layout);
    layout1->addWidget(sim_chkbox);
    layout1->addWidget(exec_pipeline_frame);
    layout1->addWidget(model_adapt_chkbox);
    layout1->addWidget(phase_stop_chkbox);
    layout1->addWidget(reset_model_chkbox);
    layout1->addStretch(0);

  QFrame *frame1 = new QFrame(central_widget);
    frame1->setFrameStyle(QFrame::Box | QFrame::Raised);
    frame1->setLineWidth(4);
    frame1->setLayout(layout1);

  // =========== Layout 2 ===============
  QHBoxLayout *viz_target_layout = createPubFrameLayout("viz_pick_target", [this](bool is_checked, unsigned pub_rate){ctrl_->setVizualizePickTarget(is_checked, pub_rate); });
  QHBoxLayout *viz_vp_layout = createPubFrameLayout("viz_vp", [this](bool is_checked, unsigned pub_rate){ctrl_->setVizualizeViapoint(is_checked, pub_rate); });
  QHBoxLayout *viz_box_target_layout = createPubFrameLayout("viz_place_target", [this](bool is_checked, unsigned pub_rate){ctrl_->setVizualizePlaceTarget(is_checked, pub_rate); });
  QHBoxLayout *viz_obst_layout = createPubFrameLayout("viz_obst", [this](bool is_checked, unsigned pub_rate){ctrl_->setVizualizeObst(is_checked, pub_rate); });

  // ========== Layout 3 ==========
  reload_apriltag_listener_btn = new QPushButton("reload apriltag listener");
  reload_apriltag_listener_btn->setFont(font2);
  QObject::connect(reload_apriltag_listener_btn, &QPushButton::pressed, this, [this]()
  { ctrl_->reloadAprilTagListener(); });

  load_params_btn = new QPushButton("load params");
  load_params_btn->setFont(font2);
  QObject::connect(load_params_btn, &QPushButton::pressed, this, [this]()
  { showMsg(ctrl_->loadParams()); });

  set_current_pose_as_pick_target_chkbox = new QCheckBox("set current pose as pick");
  QObject::connect(set_current_pose_as_pick_target_chkbox, &QCheckBox::stateChanged, this, [this]()
  { 
    bool enable = set_current_pose_as_pick_target_chkbox->isChecked();
    ctrl_->setCurrentPoseAsPickTarget(enable); 
  });

  // ========== Layout 4 =============
  view_progress_dialog = new gui_::ViewProgressDialog(std::bind(&OnlineAdaptController::getProgress, this->ctrl_), this);
  view_progress_dialog->setWindowTitle(QString(getNodeNameID().c_str()) + ":Execution progress");
  view_exec_progress_act = new QAction("Exec progress", this);
  view_exec_progress_act->setStatusTip("Open a dialog displaying the execution progress.");
  QObject::connect( view_exec_progress_act, &QAction::triggered, this, [this]()
  { 
    view_progress_dialog->launch(); 
    if (is_ctrl_on) view_progress_dialog->startProgressBarUpdate();
  });
  QObject::connect(this, &OnlineAdaptWin::execStartedSignal, view_progress_dialog, &gui_::ViewProgressDialog::startProgressBarUpdate);
  QObject::connect(this, &OnlineAdaptWin::execStopedSignal, view_progress_dialog, &gui_::ViewProgressDialog::stopProgressBarUpdate);

  // ============ Main Layout ==============
  QVBoxLayout *main_layout = new QVBoxLayout(central_widget);
  main_layout->addWidget(frame1);
  main_layout->addLayout(viz_target_layout);
  main_layout->addLayout(viz_vp_layout);
  main_layout->addLayout(viz_box_target_layout);
  main_layout->addLayout(viz_obst_layout);
  main_layout->addWidget(load_params_btn);
  main_layout->addWidget(reload_apriltag_listener_btn);
  main_layout->addWidget(set_current_pose_as_pick_target_chkbox);
  main_layout->addStretch(0);
   
  createActions();
  createMenu();

  QObject::connect(this, &OnlineAdaptWin::showMsgSignal, this, [this](const ExecResultMsg &msg) { showMsg(msg); } );
}

QHBoxLayout *OnlineAdaptWin::createPubFrameLayout(const std::string &name, std::function<void(bool, unsigned)>callback)
{
  QLabel *pub_rate_lb = new QLabel("pub_rate (ms):");
  pub_rate_lb->setFont(font4);
  pub_rate_lb->setAlignment(Qt::AlignRight);

  QLineEdit *pub_rate_le = new QLineEdit("50");
  pub_rate_le->setFont(font4);
  pub_rate_le->setAlignment(Qt::AlignCenter);
  
  QCheckBox *viz_chbx = new QCheckBox(name.c_str());
  viz_chbx->setFont(font4);
  viz_chbx->setChecked(true);
  QObject::connect(viz_chbx, &QCheckBox::stateChanged, this, [this, pub_rate_le, viz_chbx, callback]()
  { 
    unsigned pub_rate = 0;
    if (viz_chbx->isChecked())
    {
      bool ok;
      pub_rate = pub_rate_le->text().toUInt(&ok);
      if (!ok) viz_chbx->setChecked(false);
    }
    callback(viz_chbx->isChecked(), pub_rate); 
  });
  emit viz_chbx->stateChanged(0);

  QHBoxLayout *viz_target_layout = new QHBoxLayout;
  viz_target_layout->addWidget(viz_chbx);
  viz_target_layout->addWidget(pub_rate_lb);
  viz_target_layout->addWidget(pub_rate_le);
  viz_target_layout->addStretch(0);

  return viz_target_layout;
}

void OnlineAdaptWin::createActions()
{
  save_exec_data_act = new QAction("Save exec data", this);
  QObject::connect( save_exec_data_act, &QAction::triggered, this, [this]()
  {
    // std::string path = QFileDialog::getOpenFileName(this, tr("Save data"), ctrl_->getDefaultPath().c_str(), "Bin files (*.bin)").toStdString();
    // if (path.empty()) return;
    showMsg(ctrl_->saveLoggedData());
  });

  load_exec_data_act = new QAction("Load pick model", this);
  QObject::connect( load_exec_data_act, &QAction::triggered, this, [this]()
  {
    std::string filename = QFileDialog::getOpenFileName(this, tr("Load pick model"), ctrl_->getDefaultPath().c_str(), "Bin files (*.bin)").toStdString();
    if (filename.empty()) return;
    showMsg(ctrl_->loadPickModel(filename));
  });

  show_markers_act = new QAction("show markers", this);
  show_markers_act->setStatusTip("Enable/Disable markers vizualization in rviz.");
  show_markers_act->setCheckable(true);
  show_markers_act->setChecked(true);
  QObject::connect( show_markers_act, &QAction::triggered, this, [this]()
  {
    static bool is_checked = true;
    is_checked = !is_checked;
    show_markers_act->setChecked(is_checked);
    ctrl_->viz->enable(is_checked);
  });

  show_obstacle_act = new QAction("show obstacle bounds", this);
  show_obstacle_act->setCheckable(true);
  show_obstacle_act->setChecked(false);
  QObject::connect( show_obstacle_act, &QAction::triggered, this, [this]()
  {
    static bool is_checked = false;
    is_checked = !is_checked;
    show_obstacle_act->setChecked(is_checked);
    ctrl_->showObstacle(is_checked);
  });

  clear_exec_markers_act = new QAction("Clear exec markers", this);
  QObject::connect( clear_exec_markers_act, &QAction::triggered, this, [this]() { ctrl_->viz->stop(); });

  save_model_act = new QAction("Save pick model", this);
  QObject::connect( save_model_act, &QAction::triggered, this, [this]()
  {
    std::string filename = QFileDialog::getSaveFileName(this, tr("Save pick model"), ctrl_->getDefaultPath().c_str(), "Bin files (*.bin)").toStdString();
    if (filename.empty()) return;
    showMsg(ctrl_->savePickModel(filename));
  });

  clear_exec_markers_act = new QAction("Clear exec markers", this);
  QObject::connect( clear_exec_markers_act, &QAction::triggered, this, [this]() { ctrl_->viz->stop(); });

  // plot_train_data_act = new QAction("demo data", this);
  // plot_train_data_act->setStatusTip("Plots the training data.");
  // QObject::connect( plot_train_data_act, &QAction::triggered, this, [this](){ std::thread([this](){ ctrl_->plotTrainData(); } ).detach(); });

  // close_all_plots_act = new QAction("close all", this);
  // close_all_plots_act->setStatusTip("Closes all figures.");
  // QObject::connect( close_all_plots_act, &QAction::triggered, this, [this]()
  // { std::thread([this](){ pl_::closeAll(); } ).detach(); });

}

void OnlineAdaptWin::createMenu()
{
  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));
    file_menu->addAction(save_exec_data_act);
    file_menu->addSeparator();
    file_menu->addAction(load_exec_data_act);
    file_menu->addAction(save_model_act);
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

  QMenu *view_menu = menu_bar->addMenu(tr("&View"));
    // QMenu *plot_menu = view_menu->addMenu(tr("&Plot"));
    //   plot_menu->addAction(plot_train_data_act);
    //   plot_menu->addAction(plot_demo_sim_data_act);
    //   plot_menu->addAction(close_all_plots_act);
    // QMenu *rviz_menu = view_menu->addMenu(tr("&Rviz"));
    //   rviz_menu->addAction(view_learned_path_act);
    //   rviz_menu->addAction(view_target_pose_act);
    view_menu->addAction(view_exec_progress_act);

  QMenu *rviz_menu = menu_bar->addMenu(tr("&Rviz"));  
    rviz_menu->addAction(show_markers_act);
    rviz_menu->addAction(show_obstacle_act);
    rviz_menu->addAction(clear_exec_markers_act);

  // QMenu *cmd_menu = menu_bar->addMenu(tr("&Command"));
  //   cmd_menu->addAction(exec_train_traj_act);
  //   cmd_menu->addAction(exec_rev_train_traj_act);
}

OnlineAdaptWin::~OnlineAdaptWin()
{
    stop();
}

void OnlineAdaptWin::launch()
{
  if (!run)
  {
    run = true;
    this->show();
  }
}


void OnlineAdaptWin::stop()
{
  if (run) run = false;
  this->hide();
}

void OnlineAdaptWin::startBtnPressed()
{
  if (is_ctrl_on) return;
  is_ctrl_on = true;
  updateGUIonStartStopBtn();
  emit execStartedSignal();

  std::thread([this](){ ctrl_->start(); }).detach();
}

void OnlineAdaptWin::stopBtnPressed()
{
  if (!is_ctrl_on) return;
  is_ctrl_on = false;
  updateGUIonStartStopBtn();
  emit execStopedSignal();
  showMsg(ctrl_->stop());
}

void OnlineAdaptWin::updateGUIonStartStopBtn()
{
  if (is_ctrl_on) start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  else start_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  stop_btn->setEnabled(is_ctrl_on);
  sim_chkbox->setEnabled(!is_ctrl_on);
}

void OnlineAdaptWin::closeEvent(QCloseEvent *event)
{
  stop();
}


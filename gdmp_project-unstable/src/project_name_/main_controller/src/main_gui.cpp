#include <ros/ros.h>
#include <ros/package.h>

#include <main_controller/main_controller.h>
#include <main_controller/controller.h>
#include <main_controller/main_gui.h>

#include <io_lib/xml_parser.h>

MainWindow *MainWindow::main_win;

MainWindow::MainWindow(MainController *main_controller, QWidget *parent) : QMainWindow(parent)
{
  main_win = this;

  qRegisterMetaType<ExecResultMsg>("ExecResultMsg");

  std::vector<std::string> priority_name = {"IdlePriority", "LowestPriority", "LowPriority", "NormalPriority", "HighPriority", "HighestPriority", "TimeCriticalPriority", "InheritPriority"};

  QThread::Priority priority = QThread::currentThread()->priority();

  PRINT_INFO_MSG("[MainWindow::MainWindow]: " + priority_name[priority] + "\n", std::cerr);

  this->ctrl_ = main_controller;

  //this->resize(400,350);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":Main window");

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  font1 = QFont("Ubuntu", 17, QFont::DemiBold);
  font2 = QFont("Ubuntu", 15, QFont::DemiBold);
  font3 = QFont("Ubuntu", 14, 57);
  font4 = QFont("Ubuntu", 12, QFont::Normal);

  // --------------------------------------------

  createActions();

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addWidget(createModeFrame());
  main_layout->addWidget(createUtilsFrame());

  createMenu();

   std::cerr << "[MainWindow]: Thread id: " << QThread::currentThreadId() << "\n";

  // ========= connections =============
  QObject::connect( this, &MainWindow::showMsgSignal, this, [](ExecResultMsg msg){ showMsg(msg); } );
  QObject::connect( this, SIGNAL(closeSignal()), this, SLOT(close()) );
  QObject::connect( this, &MainWindow::modeChangedSignal, this, [this]()
  {
    QString style_sheet;
    rw_::Mode rmode = ctrl_->robot->getMode();
    if (rmode == rw_::Mode::FREEDRIVE) style_sheet = "color:rgb(0,200,0);";
    else if (rmode == rw_::Mode::ADMITTANCE) style_sheet = "color:rgb(0,200,255);";
    else if (rmode == rw_::Mode::IDLE) style_sheet = "color:rgb(255,160,52);";
    else if (rmode == rw_::Mode::CART_VEL_CTRL) style_sheet = "color:rgb(255,65,195);";
    else if (rmode == rw_::Mode::JOINT_TORQUE_CONTROL) style_sheet = "color:rgb(105,220,255);";
    else if (rmode == rw_::Mode::JOINT_POS_CONTROL) style_sheet = "color:rgb(170,85,0);";
    else if (rmode == rw_::Mode::JOINT_VEL_CONTROL) style_sheet = "color:rgb(100,190,50);";
    else if (rmode == rw_::Mode::STOPPED) style_sheet = "color:rgb(85,0,127);";
    else
    {
      robot_mode_le->setText("---");
      robot_mode_le->setStyleSheet("color:rgb(255,0,0);");
      return;
    }
    robot_mode_le->setText(ctrl_->robot->getModeName().c_str());
    robot_mode_le->setStyleSheet(style_sheet);

    if (rmode != rw_::Mode::ADMITTANCE) adm_params_frame->setVisible(false);
  });

  QObject::connect( this, &MainWindow::emergencyStopSignal, this, &MainWindow::emergencyStopSlot );

  emit modeChangedSignal();
}

MainWindow::~MainWindow()
{}

void MainWindow::createAdmittanceMode()
{
  admittance_btn = new QPushButton("Admittance");
  admittance_btn->setFont(font1);
  
  adm_enabled_dofs = {0,0,0,0,0,0};
  QLabel *enabled_dofs_lb = new QLabel("Enabled DoFs:");
  enabled_dofs_lb->setFont(font3);
  enabled_dofs_lb->setAlignment(Qt::AlignLeft);

  QHBoxLayout *enabled_dofs_layout = new QHBoxLayout;
  std::vector<QString> dofs_lb = {"x", "y", "z", "rx", "ry", "rz"};
  int n_dofs = dofs_lb.size();
  for (int i=0; i<n_dofs; i++)
  {
    QCheckBox *chkbox = new QCheckBox(dofs_lb[i]);
    chkbox->setFont(font3);
    chkbox->setChecked(false);
    QObject::connect( chkbox, &QCheckBox::stateChanged, this, [this, chkbox, i]()
    { 
      adm_enabled_dofs[i] = chkbox->isChecked();
      ctrl_->setAdmittanceEnabledDoFs(adm_enabled_dofs);
    });
    emit chkbox->stateChanged(true); // for initialization
    enabled_dofs_layout->addWidget(chkbox);
    adm_dofs_chkbox.push_back(chkbox);
  }
  enabled_dofs_layout->addStretch(0);
  QObject::connect( admittance_btn, &QPushButton::clicked, this, [this]()
  {
    static bool clicked = false;

    if (this->ctrl_->getMode() == rw_::Mode::ADMITTANCE) clicked = !clicked;
    else
    {
      clicked = true;
      this->ctrl_->setMode(rw_::Mode::ADMITTANCE);

      adm_enabled_dofs = {0,0,0,0,0,0};
      for (int i=0; i<adm_dofs_chkbox.size(); i++)  adm_enabled_dofs[i] = adm_dofs_chkbox[i]->isChecked();

      arma::uvec v = {adm_enabled_dofs[0], adm_enabled_dofs[1], adm_enabled_dofs[2], adm_enabled_dofs[3], adm_enabled_dofs[4], adm_enabled_dofs[5]};
      ctrl_->setAdmittanceEnabledDoFs(adm_enabled_dofs);
    }
    adm_params_frame->setVisible(clicked);
  });

  adm_params_frame = new QFrame;
  QVBoxLayout *adm_params_layout = new QVBoxLayout;
  adm_params_layout->addWidget(enabled_dofs_lb);
  adm_params_layout->addLayout(enabled_dofs_layout);
  adm_params_frame->setLayout(adm_params_layout);
  adm_params_frame->setVisible(false);

  adm_frame = new QFrame;
  QVBoxLayout *adm_layout = new QVBoxLayout;
  adm_layout->addWidget(admittance_btn);
  adm_layout->addWidget(adm_params_frame);
  adm_frame->setLayout(adm_layout);
}

QFrame *MainWindow::createModeFrame()
{
  freedrive_btn = new QPushButton("Freedrive");
  freedrive_btn->setFont(font1);
  QObject::connect( freedrive_btn, &QPushButton::clicked, this, [this](){ this->ctrl_->setMode(rw_::Mode::FREEDRIVE); } );

  createAdmittanceMode();

  idle_btn = new QPushButton("Idle");
  idle_btn->setFont(font1);
  QObject::connect( idle_btn, &QPushButton::clicked, this, [this](){ this->ctrl_->setMode(rw_::Mode::IDLE); } );

  // ===========  ctrl buttons =========
  QVBoxLayout *ctrl_btns_layout = new QVBoxLayout;
  for (int i=0; i<ctrl_->controller.size(); i++)
  {
    QPushButton *btn = ctrl_->controller[i]->createGui(this);
    btn->setFont(font1);
    ctrl_btns_layout->addWidget(btn);
  }
  QFrame *ctrl_btns_frame = new QFrame();
  ctrl_btns_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  ctrl_btns_frame->setLineWidth(2);
  ctrl_btns_frame->setLayout(ctrl_btns_layout);


  QLabel *robot_mode_lb = new QLabel("Robot mode");
  robot_mode_lb->setAlignment(Qt::AlignCenter);
  robot_mode_lb->setFont(font1);
  robot_mode_lb->setStyleSheet("color:rgb(0,0,255); background-color:rgba(210, 210, 210, 100);");
  robot_mode_le = new QLineEdit("");
  robot_mode_le->setAlignment(Qt::AlignCenter);
  robot_mode_le->setFont(font1);
  robot_mode_le->setMinimumWidth(300);
  robot_mode_le->setReadOnly(true);

  QVBoxLayout *robot_mode_layout = new QVBoxLayout;
  robot_mode_layout->addWidget(robot_mode_lb);
  robot_mode_layout->addWidget(robot_mode_le);
  QFrame *robot_mode_frame = new QFrame(central_widget);
  robot_mode_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  robot_mode_frame->setLineWidth(2);
  robot_mode_frame->setLayout(robot_mode_layout);

  QVBoxLayout *mode_layout = new QVBoxLayout;
  mode_layout->addWidget(robot_mode_frame);
  mode_layout->addWidget(freedrive_btn);
  mode_layout->addWidget(idle_btn);
  mode_layout->addWidget(adm_frame);
  mode_layout->addWidget(ctrl_btns_frame);
  mode_layout->addStretch(0);

  QFrame *mode_frame = new QFrame(central_widget);
  mode_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  mode_frame->setLineWidth(4);
  mode_frame->setLayout(mode_layout);

  return mode_frame;
}

QFrame *MainWindow::createUtilsFrame()
{
  emerg_stop_btn = new QPushButton("Emergency stop");
  emerg_stop_btn->setFont(font1);
  emerg_stop_btn->setMinimumSize(80,80);
  emerg_stop_btn->setIcon(QIcon(":/panic_button_icon"));
  emerg_stop_btn->setIconSize(QSize(50,50));
  emerg_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(210, 210, 210, 100);");
  QObject::connect( emerg_stop_btn, &QPushButton::pressed, this, &MainWindow::emergencyStopSlot);

  enable_robot_btn = new QPushButton("Enable robot");
  enable_robot_btn->setFont(font1);
  enable_robot_btn->setStyleSheet("color:rgb(0,0,250); background-color:rgba(100, 180, 100, 100);");
  enable_robot_btn->setVisible(false);
  QObject::connect( enable_robot_btn, &QPushButton::pressed, this, [this]()
  {
    enable_robot_btn->setVisible(false);
    emerg_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(210, 210, 210, 100);");
    ctrl_->robot->setExternalStop(false);
    ctrl_->setMode(rw_::IDLE);
    emit modeChangedSignal();
    emit this->showMsgSignal(ExecResultMsg(ExecResultMsg::INFO, "The robot is enabled!"));
  });

  goto_start_btn = new QPushButton("Goto start joints pos");
  goto_start_btn->setFont(font1);
  QObject::connect( goto_start_btn, &QPushButton::pressed, this, [this]()
  {
    updateGUIonGotoPose(false);
    std::thread ctrl_thr = std::thread([this]() { emit this->reachedPoseSignal(ctrl_->gotoStartPose()); });
    ctrl_->makeThreadRT(ctrl_thr);
    ctrl_thr.detach();
  });
  QObject::connect( this, &MainWindow::reachedPoseSignal, this, [this](ExecResultMsg msg)
  { updateGUIonGotoPose(true); showMsg(msg); });

  goto_start_pose_btn = new QPushButton("Goto start pose");
  goto_start_pose_btn->setFont(font1);
  QObject::connect( goto_start_pose_btn, &QPushButton::pressed, this, [this]()
  {
    updateGUIonGotoPose(false);
    std::thread ctrl_thr = std::thread([this]() { emit this->reachedPoseSignal(ctrl_->gotoStartPose(false)); });
    ctrl_->makeThreadRT(ctrl_thr);
    ctrl_thr.detach();
  });
  QObject::connect( this, &MainWindow::reachedPoseSignal, this, [this](ExecResultMsg msg)
  { updateGUIonGotoPose(true); showMsg(msg); });

  set_current_start_btn = new QPushButton("Set current pose as start");
  set_current_start_btn->setFont(font1);
  QObject::connect( set_current_start_btn, &QPushButton::pressed, this, [this]()
  {
    ExecResultMsg msg = ctrl_->setCurrentPoseAsStart();
    emit this->startPoseChangedSignal(ctrl_->getStartPose(), msg);
  });

  QObject::connect( this, &MainWindow::startPoseChangedSignal, this, [this](const arma::vec &q_start, ExecResultMsg msg)
  {
    emit this->set_start_pose_dialog->setJointsPosSignal(q_start);
    showMsg(msg);
  });

  goto_jpos_btn = new QPushButton("Goto joints pos:");
  goto_jpos_btn->setFont(font1);
  QObject::connect( goto_jpos_btn, &QPushButton::pressed, this, [this]()
  {
    arma::vec jpos = arma::vec(getNumbersFromString(goto_jpos_le->text().toStdString()));
    unsigned n_joints = ctrl_->robot->getNumOfJoints();
    if (jpos.size() != n_joints)
    {
      showMsg(ExecResultMsg(ExecResultMsg::WARNING, "Joint pos must have exactly " + std::to_string(n_joints) + " numbers separated by comma or semicolon"));
      return;
    }
    updateGUIonGotoPose(false);
    std::thread ctrl_thr = std::thread([this, jpos](){ emit this->reachedPoseSignal(ctrl_->moveToJointsPosition(jpos)); });
    ctrl_->makeThreadRT(ctrl_thr);
    ctrl_thr.detach();
  });
  goto_jpos_le = new QLineEdit("");
  goto_jpos_le->setFont(font2);
  goto_jpos_le->setAlignment(Qt::AlignCenter);
  QVBoxLayout *goto_jpos_layout = new QVBoxLayout();
  goto_jpos_layout->addWidget(goto_jpos_btn);
  goto_jpos_layout->addWidget(goto_jpos_le);
  goto_jpos_layout->addStretch(0);
  QFrame *goto_jpos_frame = new QFrame();
  goto_jpos_frame->setLayout(goto_jpos_layout);
  goto_jpos_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  goto_jpos_frame->setLineWidth(2);
  goto_jpos_frame->setLayout(goto_jpos_layout);


  goto_pose_btn = new QPushButton("Goto pose:");
  goto_pose_btn->setFont(font1);
  QObject::connect(goto_pose_btn, &QPushButton::pressed, this, [this]()
  {
    std::vector<double> pose = getNumbersFromString(goto_pose_le->text().toStdString());
    if (pose.size() != 7)
    {
      showMsg(ExecResultMsg(ExecResultMsg::WARNING, "pose must have exactly 7 numbers separated by comma or semicolon"));
      return;
    }
    arma::vec goto_pos = {pose[0], pose[1], pose[2]};
    arma::vec goto_quat = {pose[3], pose[4], pose[5], pose[6]};
    if (std::fabs(arma::norm(goto_quat) - 1) > 1e-2)
    {
      goto_quat = goto_quat / arma::norm(goto_quat);
      showMsg(ExecResultMsg(ExecResultMsg::WARNING, "Non unit quaternion. Will normalize it..."));
    }
    
    updateGUIonGotoPose(false);
    std::thread ctrl_thr = std::thread([this, goto_pos, goto_quat](){ emit this->reachedPoseSignal(ctrl_->moveToCartPose(goto_pos, goto_quat)); });
    ctrl_->makeThreadRT(ctrl_thr);
    ctrl_thr.detach();
  });
  goto_pose_le = new QLineEdit("");
  goto_pose_le->setFont(font2);
  goto_pose_le->setAlignment(Qt::AlignCenter);
  QVBoxLayout *goto_pose_layout = new QVBoxLayout();
  goto_pose_layout->addWidget(goto_pose_btn);
  goto_pose_layout->addWidget(goto_pose_le);
  goto_pose_layout->addStretch(0);
  QFrame *goto_pose_frame = new QFrame();
  goto_pose_frame->setLayout(goto_pose_layout);
  goto_pose_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  goto_pose_frame->setLineWidth(2);
  goto_pose_frame->setLayout(goto_pose_layout);

  // replay_recorded_motion_btn = new QPushButton("Replay recorded motion");
  // replay_recorded_motion_btn->setFont(font1);
  // QObject::connect( replay_recorded_motion_btn, &QPushButton::pressed, this, [this]()
  // {
  //   updateGUIonReplayRecMotion(false);
  //   std::thread ctrl_thr = std::thread([this]() { emit this->finishedMotionReplay(ctrl_->replayRecordedMotion()); });
  //   ctrl_->makeThreadRT(ctrl_thr);
  //   ctrl_thr.detach();
  // });
  // QObject::connect( this, &MainWindow::finishedMotionReplay, this, [this](ExecResultMsg msg)
  // { updateGUIonReplayRecMotion(true); showMsg(msg); });

  QVBoxLayout *utils_layout = new QVBoxLayout;
  utils_layout->addWidget(emerg_stop_btn);
  utils_layout->addWidget(enable_robot_btn);
  utils_layout->addWidget(goto_start_btn);
  utils_layout->addWidget(goto_start_pose_btn);
  utils_layout->addWidget(goto_jpos_frame);
  utils_layout->addWidget(goto_pose_frame);
  utils_layout->addWidget(set_current_start_btn);
  // utils_layout->addWidget(replay_recorded_motion_btn);
  utils_layout->addStretch(0);

  QFrame *utils_frame = new QFrame(central_widget);
  utils_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  utils_frame->setLineWidth(4);
  utils_frame->setLayout(utils_layout);

  return utils_frame;
}

std::vector<double> MainWindow::getNumbersFromString(const std::string &str)
{
  std::string nums_str = str;
  std::replace(nums_str.begin(), nums_str.end(), ',', ' ');
  std::replace(nums_str.begin(), nums_str.end(), ';', ' ');
  std::istringstream iss(nums_str);
  std::vector<double> nums;
  while (true)
  {
    double a; iss >> a; if (!iss) break;
    nums.push_back(a);
  }
  return nums;
}

void MainWindow::createActions()
{
  // ===============  Edit menu actions  ================
  set_start_pose_dialog = new gui_::SetJointsPosDialog(std::bind(&MainController::getStartPose, ctrl_), std::bind(&MainController::setStartPose, ctrl_, std::placeholders::_1), this);
  set_start_pose_act = new QAction(tr("Set start pose"), this);
  set_start_pose_act->setStatusTip(tr("Opens a dialog where you can set the start pose."));
  QObject::connect( set_start_pose_act, &QAction::triggered, this, [this](){ set_start_pose_dialog->launch(); } );

  bias_FTsensor_act = new QAction(tr("Bias FT-sensor"), this);
  bias_FTsensor_act->setStatusTip(tr("Bias the FT-sensor."));
  QObject::connect( bias_FTsensor_act, &QAction::triggered, [this](){ showMsg( ctrl_->biasFTsensor() ); } );

  // ===============  View menu actions  ================
  gui_::ViewJPosDialog *view_jpos_dialog = new gui_::ViewJPosDialog(ctrl_->robot->getJointPosLowLim(), ctrl_->robot->getJointPosUpperLim(), std::bind(&rw_::Robot::getJointsPosition, ctrl_->robot), this);
  view_jpos_dialog->setJointNames(ctrl_->robot->getJointNames());
  view_jpos_dialog->setTitle("Robot joints positions");
  view_joints_act = new QAction(tr("View joints"), this);
  view_joints_act->setStatusTip(tr("Opens a window with sliders displaying the robot's joints position."));
  QObject::connect( view_joints_act, &QAction::triggered, this, [view_jpos_dialog](){ view_jpos_dialog->launch(); } );


  view_start_pose_act = new QAction(tr("start pose"), this);
  view_start_pose_act->setStatusTip(tr("Visualizes the start pose frame in rviz."));
  view_start_pose_act->setCheckable(true);
  view_start_pose_act->setChecked(false);
  QObject::connect( view_start_pose_act, &QAction::triggered, this, [this]()
  { 
    static bool is_checked = false;

    is_checked = !is_checked;
    view_start_pose_act->setChecked(is_checked);
    ctrl_->viewStartPose(is_checked); 
  });

  view_start_joint_pos_act = new QAction(tr("start joint pos"), this);
  view_start_joint_pos_act->setStatusTip(tr("Visualizes the start joints position frame in rviz."));
  view_start_joint_pos_act->setCheckable(true);
  view_start_joint_pos_act->setChecked(false);
  QObject::connect( view_start_joint_pos_act, &QAction::triggered, this, [this]()
  { 
    static bool is_checked = false;

    is_checked = !is_checked;
    view_start_joint_pos_act->setChecked(is_checked);
    ctrl_->viewStartJointPos(is_checked); 
  });

  // If I define this 'inline' as an argument, the function malfunctions...??????
  auto getTaskPoseFun = [this]()
  {
    arma::vec pose = arma::join_vert(ctrl_->robot->getTaskPosition(), ctrl_->robot->getTaskOrientation());
    return pose;
  };
  gui_::ViewPoseDialog *view_pose_dialog = new gui_::ViewPoseDialog(getTaskPoseFun, this);
  view_pose_dialog->setTitle("Tool pose");
  view_pose_act = new QAction(tr("View pose"), this);
  view_pose_act->setStatusTip(tr("Opens a window displaying the robot's end-effector pose."));
  QObject::connect( view_pose_act, &QAction::triggered, this, [view_pose_dialog](){ view_pose_dialog->launch();} );

  std::map< std::string, std::function<arma::vec()> > wrench_map;
  wrench_map["base"] = [this](){ return ctrl_->robot->getCompTaskWrench(); };
  wrench_map["tool"] = [this]()
  {
    arma::vec wrench = ctrl_->robot->getCompTaskWrench();
    arma::mat R = ctrl_->robot->getTaskRotMat().t();
    wrench.subvec(0,2) = R*wrench.subvec(0,2);
    wrench.subvec(3,5) = R*wrench.subvec(3,5);
    return wrench;
  };
  gui_::ViewWrenchDialog *view_wrench_dialog = new gui_::ViewWrenchDialog(wrench_map, this);
  // gui_::ViewWrenchDialog *view_wrench_dialog = new gui_::ViewWrenchDialog([this](){ return ctrl_->robot->getCompTaskWrench(); }, [this](){ return ctrl_->robot->getTaskRotMat(); }, this);
  view_wrench_dialog->setTitle("Tool wrench");
  view_wrench_act = new QAction(tr("View wrench"), this);
  view_wrench_act->setStatusTip(tr("Opens a window displaying the compensated tool wrench."));
  QObject::connect( view_wrench_act, &QAction::triggered, this, [view_wrench_dialog](){ view_wrench_dialog->launch(); } );

  // ===============  Util menu actions  ================
  print_current_pose_act = new QAction("print tool pose", this);
  print_current_pose_act->setStatusTip("Prints the current robot tool pose to the terminal.");
  QObject::connect( print_current_pose_act, &QAction::triggered, this, [this](){ ctrl_->printToolPose(); });

  print_joints_pos_act = new QAction("print joints position", this);
  print_joints_pos_act->setStatusTip("Prints the current robot joints position to the terminal.");
  QObject::connect( print_joints_pos_act, &QAction::triggered, this, [this](){ ctrl_->printJointsPosition(); });

  joints_rec_win = new RobotJointsRecordWin(ctrl_->joints_rec_.get(), this);
  joints_rec_win_act = new QAction("Robot joints recorder", this);
  joints_rec_win_act->setStatusTip("Save model from disk.");
  QObject::connect( joints_rec_win_act, &QAction::triggered, this, [this](){ joints_rec_win->show(); });

  on_robot_cam_callib_act = new QAction("On-robot camera callibration", this);
  on_robot_cam_callib_act->setStatusTip("Performs robot-camera callibration using an on-robot april tag.");
  QObject::connect( on_robot_cam_callib_act, &QAction::triggered, this, [this]()
  { std::thread([this](){ ctrl_->onRobotCameraCallibration(); }).detach(); });
}

void MainWindow::createMenu()
{
  // =======   Create menus   ==========

  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));

  QMenu *edit_menu = menu_bar->addMenu(tr("&Edit"));
  edit_menu->addAction(set_start_pose_act);
  edit_menu->addSeparator();
  edit_menu->addAction(bias_FTsensor_act);

  QMenu *view_menu = menu_bar->addMenu(tr("&View"));
  view_menu->addAction(view_joints_act);
  view_menu->addAction(view_pose_act);
  view_menu->addAction(view_wrench_act);
  QMenu *rviz_menu = view_menu->addMenu("Rviz");
  rviz_menu->addAction(view_start_pose_act);
  rviz_menu->addAction(view_start_joint_pos_act);

  QMenu *utils_menu = menu_bar->addMenu(tr("&Utils"));
  utils_menu->addAction(print_current_pose_act);
  utils_menu->addAction(print_joints_pos_act);
  utils_menu->addAction(joints_rec_win_act);
  utils_menu->addAction(on_robot_cam_callib_act);
}

void MainWindow::updateGUIonGotoPose(bool set)
{
  goto_start_btn->setEnabled(set);
  set_current_start_btn->setEnabled(set);
  freedrive_btn->setEnabled(set);
  idle_btn->setEnabled(set);
  admittance_btn->setEnabled(set);

  set_start_pose_act->setEnabled(set);
}

void MainWindow::updateGUIonReplayRecMotion(bool set)
{
  goto_start_btn->setEnabled(set);
  set_current_start_btn->setEnabled(set);
  freedrive_btn->setEnabled(set);
  admittance_btn->setEnabled(set);
  idle_btn->setEnabled(set);

  set_start_pose_act->setEnabled(set);
}

void MainWindow::emergencyStopSlot()
{
  emerg_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(180, 100, 100, 150);");
  this->setEnabled(false);
  ctrl_->robot->setExternalStop(true);
  robot_mode_le->setText("EMERGENCY STOP");
  robot_mode_le->setStyleSheet("color:rgb(255,0,0);");
  enable_robot_btn->setVisible(true);
  // emit this->showMsgSignal(ExecResultMsg(ExecResultMsg::WARNING, "Emergency stop activated!"));
  this->setEnabled(true);
}

// ============== Show Msg ==================

int MainWindow::showMsg(const ExecResultMsg &msg)
{
  switch (msg.getType())
  {
    case ExecResultMsg::INFO:
      return MainWindow::showInfoMsg(msg.getMsg().c_str());
    case ExecResultMsg::WARNING:
      return MainWindow::showWarningMsg(msg.getMsg().c_str());
    case ExecResultMsg::ERROR:
      return MainWindow::showErrorMsg(msg.getMsg().c_str());
    case ExecResultMsg::QUESTION:
      return MainWindow::showQuestionMsg(msg.getMsg().c_str());
    default:
      throw std::runtime_error("Invalid msg type ...");
  }
}

int MainWindow::showErrorMsg(const QString &msg)
{
  QMessageBox msg_box;

  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Critical);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.setModal(true);

  return msg_box.exec();
}

int MainWindow::showWarningMsg(const QString &msg)
{
  QMessageBox msg_box;

  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Warning);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.setModal(true);

  return msg_box.exec();
}

int MainWindow::showQuestionMsg(const QString &msg)
{
  QMessageBox msg_box;

  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Question);
  msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
  msg_box.setModal(true);

  return msg_box.exec();
}

int MainWindow::showInfoMsg(const QString &msg)
{
  QMessageBox msg_box;

  msg_box.setText(msg);
  msg_box.setIcon(QMessageBox::Information);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.setModal(true);

  return msg_box.exec();
}

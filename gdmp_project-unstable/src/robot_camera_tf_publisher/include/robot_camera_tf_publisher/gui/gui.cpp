#include <ros/ros.h>
#include <ros/package.h>

#include <robot_camera_tf_publisher/controller.h>
#include <robot_camera_tf_publisher/gui/gui.h>
#include <robot_camera_tf_publisher/print_utils.h>

#include <thread_lib/thread_lib.h>

MainWindow *MainWindow::main_win;

MainWindow::MainWindow(Controller *main_controller, QWidget *parent) : QMainWindow(parent)
{
  qRegisterMetaType<MSG_TYPE>("MSG_TYPE");

  main_win = this;

  std::vector<std::string> priority_name = {"IdlePriority", "LowestPriority", "LowPriority", "NormalPriority", "HighPriority", "HighestPriority", "TimeCriticalPriority", "InheritPriority"};

  QThread::Priority priority = QThread::currentThread()->priority();

  PRINT_INFO_MSG("[MainWindow::MainWindow]: " + priority_name[priority] + "\n", std::cerr);

  this->main_ctrl = main_controller;

  //this->resize(400,350);
  this->setWindowTitle(QString("Robot-camera tf publisher"));

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  font1 = QFont("Ubuntu", 15, QFont::DemiBold);
  font2 = QFont("Ubuntu", 13, QFont::DemiBold);
  
  // --------------------------------------------

  QVBoxLayout *main_layout = new QVBoxLayout(central_widget);
  main_layout->addWidget(createModeFrame());
  main_layout->addWidget(createUtilsFrame());

  createMenu();

  // ========= connections =============
  QObject::connect( this, SIGNAL(closeSignal()), this, SLOT(close()) );
  QObject::connect( this, &MainWindow::showMsgSignal, this, [](const QString &msg, enum MSG_TYPE msg_type)
  { 
    if (msg_type == INFO_MSG) showInfoMsg(msg);
    else if (msg_type == WARN_MSG) showWarningMsg(msg);
    else if (msg_type == ERR_MSG) showErrorMsg(msg);
  });
  

  emit modeChangedSignal();
}

MainWindow::~MainWindow()
{}

void MainWindow::createMenu()
{
  print_joints_pos_act = new QAction("Print joints pos", this);
  QObject::connect( print_joints_pos_act, &QAction::triggered, this, [this]()
  { main_ctrl->printJointsPos(); });

  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *utils_menu = menu_bar->addMenu(tr("&Utils"));
  utils_menu->addAction(print_joints_pos_act);
}

QFrame *MainWindow::createModeFrame()
{
  freedrive_btn = new QPushButton("Freedrive");
  freedrive_btn->setFont(font1);
  QObject::connect( freedrive_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->setMode(rw_::Mode::FREEDRIVE); } );

  idle_btn = new QPushButton("Idle");
  idle_btn->setFont(font1);
  QObject::connect( idle_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->setMode(rw_::Mode::IDLE); } );

  QLabel *robot_mode_lb = new QLabel("Robot mode");
  robot_mode_lb->setAlignment(Qt::AlignCenter);
  robot_mode_lb->setFont(font1);
  robot_mode_lb->setStyleSheet("color:rgb(0,0,255); background-color:rgba(210, 210, 210, 100);");
  robot_mode_le = new QLineEdit("");
  robot_mode_le->setAlignment(Qt::AlignCenter);
  robot_mode_le->setFont(font1);
  robot_mode_le->setMinimumWidth(300);
  robot_mode_le->setReadOnly(true);
  QObject::connect( this, &MainWindow::modeChangedSignal, this, [this]()
  {
    QString style_sheet;
    rw_::Mode rmode = main_ctrl->getRobotMode();
    if (rmode == rw_::Mode::FREEDRIVE) style_sheet = "color:rgb(0,200,0);";
    else if (rmode == rw_::Mode::IDLE) style_sheet = "color:rgb(255,160,52);";
    else if (rmode == rw_::Mode::JOINT_POS_CONTROL) style_sheet = "color:rgb(170,85,0);";
    else
    {
      robot_mode_le->setText("---");
      robot_mode_le->setStyleSheet("color:rgb(255,0,0);");
      return;
    }
    robot_mode_le->setText(main_ctrl->getRobotModeName().c_str());
    robot_mode_le->setStyleSheet(style_sheet);
  });

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
  mode_layout->addStretch(0);

  QFrame *mode_frame = new QFrame(central_widget);
  mode_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  mode_frame->setLineWidth(4);
  mode_frame->setLayout(mode_layout);

  return mode_frame;
}

QFrame *MainWindow::createUtilsFrame()
{
  // =========== publish robot-camera tf btn =============

  publish_robot_cam_tf_btn = new QPushButton("publish robot-camera tf");
  publish_robot_cam_tf_btn->setFont(font1);
  QObject::connect( publish_robot_cam_tf_btn, &QPushButton::clicked, this, [this](){ this->main_ctrl->publishRobotCameraTf(); } );

  auto_pub_chkbox = new QCheckBox("auto-publish tf");
  auto_pub_chkbox->setChecked(true);
  auto_pub_chkbox->setFont(font1);
  QObject::connect( auto_pub_chkbox, &QCheckBox::stateChanged, this, [this]()
  { main_ctrl->enableAutoPublish(auto_pub_chkbox->isChecked()); });
  emit auto_pub_chkbox->stateChanged(Qt::Checked);

  viz_published_tf_in_rviz_chkbox = new QCheckBox("visualize the published tf");
  viz_published_tf_in_rviz_chkbox->setChecked(false);
  viz_published_tf_in_rviz_chkbox->setFont(font1);
  QObject::connect( viz_published_tf_in_rviz_chkbox, &QCheckBox::stateChanged, this, [this]()
  { main_ctrl->vizPublishedTf(viz_published_tf_in_rviz_chkbox->isChecked()); });

  
  
  // =========== move to joints position frame =============
  QFrame *move2jointpos_frame = new QFrame;
  {
    QLabel *move2jointpos_lb = new QLabel("joints position:");
    move2jointpos_lb->setAlignment(Qt::AlignCenter);
    move2jointpos_lb->setFont(font1);
    move2jointpos_lb->setStyleSheet("color:rgb(0,0,255); background-color:rgba(210, 210, 210, 100);");
    move2jointpos_le = new QLineEdit();
    move2jointpos_le->setAlignment(Qt::AlignCenter);
    move2jointpos_le->setFont(font1);
    move2jointpos_le->setMinimumWidth(300);
    QObject::connect( move2jointpos_le, &QLineEdit::editingFinished, this, [this]()
    {
      std::string joint_pos_str = move2jointpos_le->text().toStdString();
      std::replace( joint_pos_str.begin(), joint_pos_str.end(), ',', ' '); 
      std::istringstream iss(joint_pos_str);
      std::vector<double> joint_pos;
      while (true)
      {
        double pos;
        iss >> pos;
        if (!iss) break;
        joint_pos.push_back(pos);
      }
      move2_jointpos = joint_pos;
    });
    // initialize joint pos lineEdit with current joints position
    std::ostringstream oss;
    oss.precision(3);
    arma::vec joint_pos = main_ctrl->getRobotJointsPosition();
    for (int i=0; i<joint_pos.size()-1; i++) oss << joint_pos(i) << " , ";
    oss << joint_pos.back();
    move2jointpos_le->setText(oss.str().c_str());
    emit move2jointpos_le->editingFinished();

    move2jointpos_btn = new QPushButton("move to joints position");
    move2jointpos_btn->setFont(font1);
    QObject::connect( move2jointpos_btn, &QPushButton::clicked, this, [this]()
    { main_ctrl->move2JointPosRTthread(move2_jointpos); });
    
    QVBoxLayout *move2jointpos_layout = new QVBoxLayout;
    move2jointpos_layout->addWidget(move2jointpos_lb);
    move2jointpos_layout->addWidget(move2jointpos_le);
    move2jointpos_layout->addWidget(move2jointpos_btn);
    move2jointpos_layout->addStretch(0);

    move2jointpos_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
    move2jointpos_frame->setLineWidth(2);
    move2jointpos_frame->setLayout(move2jointpos_layout);
  }

  // =================  main layout ===================

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(publish_robot_cam_tf_btn);
  layout->addWidget(auto_pub_chkbox);
  layout->addWidget(viz_published_tf_in_rviz_chkbox);
  layout->addWidget(move2jointpos_frame);
  layout->addStretch(0);

  QFrame *frame = new QFrame(central_widget);
  frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  frame->setLineWidth(4);
  frame->setLayout(layout);

  return frame;
}

// ============== Show Msg ==================
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

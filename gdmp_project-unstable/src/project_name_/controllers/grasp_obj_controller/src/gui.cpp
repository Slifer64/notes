#include <grasp_obj_controller/gui.h>
#include <grasp_obj_controller/controller.h>

#include <QDebug>
#include <iostream>
#include <io_lib/xml_parser.h>
#include <plot_lib/qt_plot.h>

#include <ros/package.h>

#define GraspObjGui_fun_ std::string("\33[1;31m[GraspObjGui::") + __func__ + "]: \33[0m"

GraspObjGui::GraspObjGui(GraspObjController *ctrl, MainWindow *parent): QMainWindow(parent)
{
  this->main_win_ = parent;
  this->ctrl_ = ctrl;

  this->ctrl_->gui = this;

  this->resize(200, 200);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":Grasp object ctrl");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  // status_bar = new QStatusBar(this);
  // this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  loadGUIParams();

  run = false;

  is_exec_on = false;

  // =============  Fonts  ================

  font1 = QFont("Ubuntu", 17, QFont::DemiBold);
  font2 = QFont("Ubuntu", 15, QFont::DemiBold);
  font3 = QFont("Ubuntu", 14, 57);
  font4 = QFont("Ubuntu", 12, QFont::Normal);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  QFrame *start_stop_frame = createStartStopFrame();

  read_img_btn = new QPushButton("read image");
  read_img_btn->setFont(font1);
  QObject::connect( read_img_btn, &QPushButton::pressed, this, [this]()
  {
    std::thread([this]()
    {
      ctrl_->readImage();
    }).detach();
  });

  move_to_start_btn = new QPushButton("move to start");
  move_to_start_btn->setFont(font1);
  QObject::connect( move_to_start_btn, &QPushButton::pressed, this, [this]()
  {
    std::thread([this]()
    {
      emit showMsgSignal(ctrl_->moveToStartPose());
    }).detach();
  });
  
  retract_btn = new QPushButton("retract");
  retract_btn->setFont(font1);
  QObject::connect( retract_btn, &QPushButton::pressed, this, [this]()
  { 
    if (is_exec_on) return;
    is_exec_on = true;
    updateGUIonStartStopBtn();
    emit execStartedSignal();
    std::thread([this]()
    {
      ctrl_->retract();
    }).detach();
  });

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  autorecord_chkbox = new QCheckBox("auto-rec");
  autorecord_chkbox->setFont(font1);
  autorecord_chkbox->setChecked(false);

  record_btn = new QPushButton("record");
  record_btn->setFont(font1);
  QObject::connect( record_btn, &QPushButton::pressed, this, [this]()
  { std::thread([this](){ ctrl_->record(); }).detach(); });
  
  QHBoxLayout *record_layout = new QHBoxLayout();
  record_layout->addWidget(autorecord_chkbox);
  record_layout->addWidget(record_btn);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  QLabel *model_lb = new QLabel("Model name:");
  model_lb->setFont(font1);
  model_cmbox = new QComboBox();
  model_cmbox->setFont(font2);
  for (auto &name : model_names) model_cmbox->addItem(name.c_str());
  QHBoxLayout *model_layout = new QHBoxLayout;
  model_layout->addWidget(model_lb);
  model_layout->addWidget(model_cmbox);
  model_layout->addStretch(0);

  QLabel *target_obj_lb = new QLabel("Target object:");
  target_obj_lb->setFont(font1);
  target_obj_cmbox = new QComboBox();
  target_obj_cmbox->setFont(font2);
  for (auto it=target_obj_map.begin(); it!=target_obj_map.end(); it++) target_obj_cmbox->addItem(it->first.c_str());
  QHBoxLayout *target_obj_layout = new QHBoxLayout;
  target_obj_layout->addWidget(target_obj_lb);
  target_obj_layout->addWidget(target_obj_cmbox);
  target_obj_layout->addStretch(0);

  msg_box = new TextBox(300);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  {
    QVBoxLayout *main_layout = new QVBoxLayout(central_widget);
    main_layout->addWidget(start_stop_frame, 0);
    main_layout->addWidget(read_img_btn);
    main_layout->addWidget(move_to_start_btn);
    main_layout->addWidget(retract_btn);
    main_layout->addLayout(record_layout);
    main_layout->addLayout(model_layout);
    main_layout->addLayout(target_obj_layout);
    main_layout->addWidget(msg_box);
    main_layout->addStretch(0);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  QObject::connect( this, &GraspObjGui::showMsgSignal, this, [this](const ExecResultMsg &msg){ showMsg(msg); });

  createActions();
  createMenu();
}

void GraspObjGui::showMsg(const ExecResultMsg &msg)
{
  QColor color(Qt::white);
  if (msg.getType() == ExecResultMsg::INFO) color = QColor(Qt::white);
  else if (msg.getType() == ExecResultMsg::WARNING) color = QColor(Qt::yellow);
  else if (msg.getType() == ExecResultMsg::ERROR) color = QColor(Qt::red);
  msg_box->setText(msg.getMsg().c_str(), color);
}

void GraspObjGui::createActions()
{
  load_params_act = new QAction("load params", this);
  load_params_act->setStatusTip("Load training params from disk.");
  QObject::connect( load_params_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load params"), ctrl_->getDefaultPath().c_str(), "Yaml files (*.yaml)").toStdString();
    if (path.empty()) return;
    std::thread([this, path]()
    {
      emit showMsgSignal(ctrl_->loadParams(path));
    }).detach();
  });

  view_exec_path_act = new QAction("3D oriented path", this);
  view_exec_path_act->setStatusTip("Publishes the target pose frame to view in rviz.");
  view_exec_path_act->setCheckable(true);
  view_exec_path_act->setChecked(false);
  QObject::connect( view_exec_path_act, &QAction::triggered, this, [this]()
  {
    static bool is_view_exec_path_act_checked = false;
    is_view_exec_path_act_checked = !is_view_exec_path_act_checked;
    view_exec_path_act->setChecked(is_view_exec_path_act_checked);
    ctrl_->viewExecPath(is_view_exec_path_act_checked);
  });
}

void GraspObjGui::createMenu()
{
  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));
    file_menu->addAction(load_params_act);

  QMenu *view_menu = menu_bar->addMenu(tr("&View"));
    QMenu *rviz_menu = view_menu->addMenu(tr("&Rviz"));
      rviz_menu->addAction(view_exec_path_act);
}

QFrame *GraspObjGui::createStartStopFrame()
{
  start_btn = new QPushButton("start");
  start_btn->setFont(font1);
  QObject::connect( start_btn, &QPushButton::pressed, this, [this](){ startBtnPressed(); }) ;

  stop_btn = new QPushButton("stop");
  stop_btn->setFont(font1);
  stop_btn->setEnabled(false);
  QObject::connect( stop_btn, &QPushButton::pressed, this, [this](){ stopBtnPressed(); }) ;
  QObject::connect( this, &GraspObjGui::stopExecSignal, this, [this](const ExecResultMsg &msg)
  {
    is_exec_on = false;
    updateGUIonStartStopBtn();
    showMsg(msg);
  });

  QLabel *train_lb = new QLabel("Start execution");
  train_lb->setFont(font1);
  train_lb->setAlignment(Qt::AlignCenter);

  QHBoxLayout *start_stop_btns_layout = new QHBoxLayout;
    start_stop_btns_layout->addWidget(start_btn);
    start_stop_btns_layout->addWidget(stop_btn);
    start_stop_btns_layout->addStretch(0);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  QVBoxLayout *train_layout = new QVBoxLayout;
    train_layout->addWidget(train_lb);
    train_layout->addLayout(start_stop_btns_layout);
    train_layout->addStretch(0);

  QFrame *train_frame = new QFrame;
    train_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
    train_frame->setLineWidth(4);
    train_frame->setLayout(train_layout);

  return train_frame;
}

GraspObjGui::~GraspObjGui()
{
    stop();
}

void GraspObjGui::launch()
{
  if (!run)
  {
    run = true;
    this->show();
  }
}


void GraspObjGui::stop()
{
  if (run) run = false;
  this->hide();
}

void GraspObjGui::startBtnPressed()
{
  if (is_exec_on) return;
  is_exec_on = true;
  updateGUIonStartStopBtn();
  emit execStartedSignal();
  std::thread([this](){ ctrl_->startExec(); }).detach();
}

void GraspObjGui::stopBtnPressed()
{
  if (!is_exec_on) return;
  is_exec_on = false;
  updateGUIonStartStopBtn();
  emit execStopedSignal();
  showMsg(ctrl_->stopExec());
}

void GraspObjGui::updateGUIonStartStopBtn()
{
  if (is_exec_on) start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  else start_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  stop_btn->setEnabled(is_exec_on);
}

void GraspObjGui::closeEvent(QCloseEvent *event)
{
  stop();
}

#include <yaml-cpp/yaml.h>

void GraspObjGui::loadGUIParams()
{
  try
  {
    std::string path = ctrl_->getDefaultPath() + "/config/models.yaml";
    YAML::Node node = YAML::LoadFile(path);
    if (node.Type() != YAML::NodeType::Map) throw std::runtime_error("models.yaml must be a map...");

    for (auto it = node.begin(); it != node.end(); ++it) 
      model_names.push_back(it->first.as<std::string>());

    path = ctrl_->getDefaultPath() + "/config/target_objects.yaml";
    node = YAML::LoadFile(path);
    if (node.Type() != YAML::NodeType::Map) throw std::runtime_error("target_objects.yaml must be a map...");
    for (auto it = node.begin(); it != node.end(); ++it)
      target_obj_map[it->first.as<std::string>()] = it->second.as<int>();
    
  }
  
  catch (std::exception &e)
  { throw std::runtime_error(GraspObjGui_fun_ + e.what()); }

}

#include <im_traj_demo_controller/gui.h>
#include <im_traj_demo_controller/controller.h>

#include <QDebug>
#include <iostream>
#include <io_lib/xml_parser.h>
#include <plot_lib/qt_plot.h>

#include <ros/package.h>

ImTragDemoGui::ImTragDemoGui(ImTrajDemoController *ctrl, MainWindow *parent): QMainWindow(parent)
{
  this->main_win_ = parent;
  this->ctrl_ = ctrl;

  this->ctrl_->gui = this;

  this->resize(200,200);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":Image - Traj Demo");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  // status_bar = new QStatusBar(this);
  // this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  run = false;

  is_train_on = false;

  // =============  Fonts  ================

  font1 = QFont("Ubuntu", 17, QFont::DemiBold);
  font2 = QFont("Ubuntu", 15, QFont::DemiBold);
  font3 = QFont("Ubuntu", 14, 57);
  font4 = QFont("Ubuntu", 12, QFont::Normal);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  QFrame *train_frame = createTrainFrame();

  capt_im_btn = new QPushButton("capture image");
  capt_im_btn->setFont(font1);
  QObject::connect( capt_im_btn, &QPushButton::pressed, this, [this]()
  { 
    ctrl_->captureImage();
    // save_demo_btn->setEnabled(ctrl_->is_demo_captured && ctrl_->is_image_captured);
  });
  
  capt_multi_im_btn = new QPushButton("capture multi image");
  capt_multi_im_btn->setFont(font1);
  QObject::connect( capt_multi_im_btn, &QPushButton::pressed, this, [this]()
  { 
    std::thread([this]()
    {
      ctrl_->captureMultiImages();
      // save_demo_btn->setEnabled(ctrl_->is_demo_captured && ctrl_->is_image_captured);
    }).detach();
  });

  clear_images_btn = new QPushButton("clear all images");
  clear_images_btn->setFont(font1);
  QObject::connect( clear_images_btn, &QPushButton::pressed, this, [this](){ ctrl_->clearImages(); });
  // ---------------------
  img_count_le = new QLineEdit("0");
  img_count_le->setFont(font1);
  img_count_le->setAlignment(Qt::AlignCenter);
  QObject::connect( this, &ImTragDemoGui::setImageCountSignal, this, [this](int c)
  { 
    img_count_le->setText(QString::number(c));
  });
  // ------------------
  QHBoxLayout *clear_img_layout = new QHBoxLayout;
  clear_img_layout->addWidget(clear_images_btn);
  clear_img_layout->addWidget(img_count_le);
  clear_img_layout->addStretch(0);

  save_demo_btn = new QPushButton("save demo");
  save_demo_btn->setFont(font1);
  // save_demo_btn->setEnabled(false);
  QObject::connect( save_demo_btn, &QPushButton::pressed, this, [this]()
  { 
    showMsg(ctrl_->saveDemo());
    // // save_demo_btn->setEnabled(false); 
  });

  QObject::connect( this, &ImTragDemoGui::abortMultiImgCaptSignal, this, [this](const char *msg)
  { showMsg(ExecResultMsg(ExecResultMsg::ERROR, msg)); });

  QLabel *save_counter_lb = new QLabel("Save counter:");
  save_counter_lb->setFont(font1);
  save_counter_le = new QLineEdit("1");
  save_counter_le->setFont(font1);
  save_counter_le->setAlignment(Qt::AlignCenter);
  QObject::connect( this, &ImTragDemoGui::setSaveCounterSignal, this, [this](int c)
  { 
    save_counter_le->setText(QString::number(c));
  });
  QHBoxLayout *save_counter_layout = new QHBoxLayout;
  save_counter_layout->addWidget(save_counter_lb);
  save_counter_layout->addWidget(save_counter_le);
  save_counter_layout->addStretch(0);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  {
    QVBoxLayout *main_layout = new QVBoxLayout(central_widget);
    main_layout->addWidget(train_frame, 0);
    main_layout->addWidget(capt_im_btn);
    main_layout->addWidget(capt_multi_im_btn);
    main_layout->addLayout(clear_img_layout);
    main_layout->addWidget(save_demo_btn);
    main_layout->addLayout(save_counter_layout);
    main_layout->addStretch(0);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  createActions();
  createMenu();

  // ============  initialize  ================

  // ExecResultMsg msg = loadParams(ros::package::getPath("train_controller") + "/config/train_gui_params.yaml");
  // msg.setMsg("[ImTragDemoGui::ImTragDemoGui]: " + msg.getMsg() + "\n");
  // if (msg.getType() == ExecResultMsg::INFO) PRINT_INFO_MSG(msg.getMsg());
  // else PRINT_ERROR_MSG(msg.getMsg());
}

void ImTragDemoGui::createActions()
{
  load_params_act = new QAction("load params", this);
  load_params_act->setStatusTip("Load training params from disk.");
  QObject::connect( load_params_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load params"), ctrl_->getDefaultPath().c_str(), "Yaml files (*.yaml)").toStdString();
    if (path.empty()) return;
    std::thread([this, path]()
    {
      ctrl_->loadParams(path);
    });
  });

  save_data_act = new QAction("Save train data", this);
  save_data_act->setStatusTip("Save training data from disk.");
  QObject::connect( save_data_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getSaveFileName(this, tr("Save training data"), ctrl_->getDefaultPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    showMsg(ctrl_->saveDemo());
  });

  view_learned_path_act = new QAction("3D oriented path", this);
  view_learned_path_act->setStatusTip("Publishes the target pose frame to view in rviz.");
  view_learned_path_act->setCheckable(true);
  view_learned_path_act->setChecked(false);
  QObject::connect( view_learned_path_act, &QAction::triggered, this, [this]()
  {
    static bool is_view_learned_path_act_checked = false;
    is_view_learned_path_act_checked = !is_view_learned_path_act_checked;
    view_learned_path_act->setChecked(is_view_learned_path_act_checked);
    ctrl_->viewLearnedPath(is_view_learned_path_act_checked);
  });

  // exec_train_traj_act = new QAction("Execute train trajectory", this);
  // exec_train_traj_act->setStatusTip("Executes the training trajectory.");
  // QObject::connect( exec_train_traj_act, &QAction::triggered, this, [this](){ std::thread([this]()
  // {
  //   view_progress_dialog->startProgressBarUpdate();
  //   ctrl_->executeTrainTrajectory(); } ).detach();
  // });

  // exec_rev_train_traj_act = new QAction("Execute reverse train trajectory", this);
  // exec_rev_train_traj_act->setStatusTip("Executes the reverse training trajectory.");
  // QObject::connect( exec_rev_train_traj_act, &QAction::triggered, this, [this](){ std::thread([this]()
  // {
  //   view_progress_dialog->startProgressBarUpdate();
  //   ctrl_->executeTrainTrajectory(true); } ).detach();
  // });

  // QObject::connect(this, &ImTragDemoGui::ctrl_execModelFinishedSignal, this, [this](const ExecResultMsg &msg)
  // {
  //   view_progress_dialog->stopProgressBarUpdate();
  //   if (!msg.empty()) showMsg(msg);
  // });

}

void ImTragDemoGui::createMenu()
{
  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));
    file_menu->addAction(load_params_act);
    file_menu->addSeparator();
    file_menu->addAction(save_data_act);

  QMenu *view_menu = menu_bar->addMenu(tr("&View"));
    QMenu *rviz_menu = view_menu->addMenu(tr("&Rviz"));
      rviz_menu->addAction(view_learned_path_act);

  // QMenu *cmd_menu = menu_bar->addMenu(tr("&Command"));
  //   cmd_menu->addAction(exec_train_traj_act);
  //   cmd_menu->addAction(exec_rev_train_traj_act);
}

QFrame *ImTragDemoGui::createTrainFrame()
{
  start_btn = new QPushButton("start");
  start_btn->setFont(font1);
  QObject::connect( start_btn, &QPushButton::pressed, this, [this](){ startBtnPressed(); }) ;

  stop_btn = new QPushButton("stop");
  stop_btn->setFont(font1);
  stop_btn->setEnabled(false);
  QObject::connect( stop_btn, &QPushButton::pressed, this, [this](){ stopBtnPressed(); }) ;
  QObject::connect( this, &ImTragDemoGui::stopTrainingSignal, this, [this](const ExecResultMsg &msg)
  {
    is_train_on = false;
    updateGUIonStartStopBtn();
    showMsg(msg);
  });

  QLabel *train_lb = new QLabel("Record demo");
  train_lb->setFont(font1);
  train_lb->setAlignment(Qt::AlignCenter);

  QHBoxLayout *start_stop_btns_layout = new QHBoxLayout;
    start_stop_btns_layout->addWidget(start_btn);
    start_stop_btns_layout->addWidget(stop_btn);
    //sst_btns_layout->insertSpacing(2,40);
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

ImTragDemoGui::~ImTragDemoGui()
{
    stop();
}

void ImTragDemoGui::launch()
{
  if (!run)
  {
    run = true;
    this->show();
  }
}


void ImTragDemoGui::stop()
{
  if (run) run = false;
  this->hide();
}

void ImTragDemoGui::startBtnPressed()
{
  if (is_train_on) return;
  is_train_on = true;
  updateGUIonStartStopBtn();
  emit execStartedSignal();
  std::thread([this](){ ctrl_->startDemoRec(); }).detach();
}

void ImTragDemoGui::stopBtnPressed()
{
  if (!is_train_on) return;
  is_train_on = false;
  updateGUIonStartStopBtn();
  emit execStopedSignal();
  showMsg(ctrl_->stopDemoRec());
}

void ImTragDemoGui::updateGUIonStartStopBtn()
{
  if (is_train_on) start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  else start_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  stop_btn->setEnabled(is_train_on);
  // save_demo_btn->setEnabled(ctrl_->is_demo_captured && ctrl_->is_image_captured);

  updateGUIonData();
}

void ImTragDemoGui::updateGUIonData()
{
  bool set = ctrl_->isData();
  save_data_act->setEnabled(set);
}

void ImTragDemoGui::closeEvent(QCloseEvent *event)
{
  stop();
}

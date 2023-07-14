#include <train_controller/train_gui.h>
#include <train_controller/train_controller.h>

#include <QDebug>
#include <iostream>
#include <io_lib/xml_parser.h>
#include <plot_lib/qt_plot.h>

#include <ros/package.h>

TrainWin::TrainWin(TrainController *ctrl, MainWindow *parent): QMainWindow(parent)
{
  this->main_win_ = parent;
  this->ctrl_ = ctrl;

  this->ctrl_->gui = this;

  this->resize(200,200);
  this->setWindowTitle(QString(getNodeNameID().c_str()) + ":GDMP training");

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
  params_frame = createParamsFrame();
  eq_vel_prof_frame = createEqulizeVelocityProfileFrame();

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  {
    QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
    main_layout->addWidget(train_frame, 0);
    QVBoxLayout *temp_layout = new QVBoxLayout;
      temp_layout->addStretch(0);
      temp_layout->addWidget(params_frame);
      temp_layout->addWidget(eq_vel_prof_frame);
      temp_layout->addStretch(0);
    main_layout->addLayout(temp_layout);
    main_layout->addStretch(0);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  createActions();
  createMenu();

  // ============  initialize  ================
  emit phase_rbtn->buttonClicked(idle_rbtn);
  emit vel_teach_method_rbtn->buttonClicked(phri_rbtn);

  updateGUIonTrainData();
  updateGUIonModel();
  updateGUIonRadioBtn();

  ExecResultMsg msg = loadParams(ros::package::getPath("train_controller") + "/config/train_gui_params.yaml");
  msg.setMsg("[TrainWin::TrainWin]: " + msg.getMsg() + "\n");
  if (msg.getType() == ExecResultMsg::INFO) PRINT_INFO_MSG(msg.getMsg());
  else PRINT_ERROR_MSG(msg.getMsg());
}

void TrainWin::createActions()
{
  load_train_data_act = new QAction("load train data", this);
  load_train_data_act->setStatusTip("Load training data from disk.");
  QObject::connect( load_train_data_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load training data"), ctrl_->getDefaultPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    ExecResultMsg msg = ctrl_->loadTrainData(path);
    updateGUIonTrainData();
    showMsg(msg);
  });

  load_model_act = new QAction("load model", this);
  load_model_act->setStatusTip("Load model from disk.");
  QObject::connect( load_model_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load model"), ctrl_->getDefaultPath().c_str(), "Binary files (*.bin)").toStdString();
    // path = FileDialog(this).open("Load model", main_ctrl->getDefaultModelPath(), "Binary files (*.bin)");
    if (path.empty()) return;
    ExecResultMsg msg;
    std::string err_msg;
    if (ctrl_->model.loadModel(path, &err_msg)) msg = ExecResultMsg(ExecResultMsg::INFO, "Load the model successfully!");
    else msg = ExecResultMsg(ExecResultMsg::ERROR, err_msg);
    updateGUIonModel();
    showMsg(msg);
  });

  save_model_act = new QAction("Save model", this);
  save_model_act->setStatusTip("Save model from disk.");
  QObject::connect( save_model_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getSaveFileName(this, tr("Save model"), ctrl_->getDefaultPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    ExecResultMsg msg;
    std::string err_msg;
    if (ctrl_->model->save(path, &err_msg)) msg = ExecResultMsg(ExecResultMsg::INFO, "Saved the model successfully!");
    else msg = ExecResultMsg(ExecResultMsg::ERROR, err_msg);
    showMsg(msg);
  });

  set_target_dialog = new gui_::SetPoseDialog(this);
  set_target_dialog->setWindowTitle(QString(getNodeNameID().c_str()) + ":Set target pose");
  set_target_pose_act = new QAction("Set target pose", this);
  set_target_pose_act->setStatusTip("Open a dialog for setting the target pose.");
  set_target_dialog->setPose(ctrl_->getTargetPose());
  QObject::connect( set_target_pose_act, &QAction::triggered, this, [this]()
  { set_target_dialog->launch(); });
  QObject::connect( set_target_dialog, &gui_::SetPoseDialog::poseChanged, this, [this](bool print_msg)
  { ctrl_->setTargetPose(set_target_dialog->getPose()); });
  QObject::connect( this, &TrainWin::setTargetPoseSignal, this, [this](const arma::vec &target_pose)
  { this->set_target_dialog->setPose(target_pose); });

  set_current_pose_as_target_act = new QAction("Set current pose as target", this);
  set_current_pose_as_target_act->setStatusTip("Sets the current robot pose as the target pose.");
  QObject::connect( set_current_pose_as_target_act, &QAction::triggered, this, [this]()
  { ctrl_->setCurrentPoseAsTarget(); });

  set_start_pose_from_train_data_act = new QAction("Set start pose from train data", this);
  set_start_pose_from_train_data_act->setStatusTip("Sets the training data start pose as start.");
  QObject::connect( set_start_pose_from_train_data_act, &QAction::triggered, this, [this]()
  { ctrl_->setStartPoseFromTrainData(); });

  set_target_pose_from_model_data_act = new QAction("Set target pose from model", this);
  set_target_pose_from_model_data_act->setStatusTip("Sets as target pose, the model's trained final pose.");
  QObject::connect( set_target_pose_from_model_data_act, &QAction::triggered, this, [this]()
  { ctrl_->setTargetPoseFromModel(); });

  load_params_act = new QAction("load params", this);
  load_params_act->setStatusTip("Load training params from disk.");
  QObject::connect( load_params_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getOpenFileName(this, tr("Load params"), ctrl_->getDefaultPath().c_str(), "Yaml files (*.yaml)").toStdString();
    if (path.empty()) return;
    showMsg(this->loadParams(path));
  });

  save_train_data_act = new QAction("Save train data", this);
  save_train_data_act->setStatusTip("Save training data from disk.");
  QObject::connect( save_train_data_act, &QAction::triggered, this, [this]()
  {
    std::string path = QFileDialog::getSaveFileName(this, tr("Save training data"), ctrl_->getDefaultPath().c_str(), "Binary files (*.bin)").toStdString();
    if (path.empty()) return;
    showMsg(ctrl_->saveTrainData(path));
  });

  trim_data_act = new QAction("Trim train data", this);
  trim_data_act->setStatusTip("Opens a dialog for trimming the training data.");
  QObject::connect( trim_data_act, &QAction::triggered, [this](){ trim_data_dialog->lauch(); } );
  trim_data_dialog = new TrimDataDialog(this);
  QObject::connect( trim_data_dialog, &TrimDataDialog::applyTrimSignal, this, [this]()
  { showMsg(ctrl_->trimDemoData(trim_data_dialog->getVelTrimThres(), trim_data_dialog->getRotVelTrimThres())); });

  remove_stops_act = new QAction("Remove stops from demo", this);
  remove_stops_act->setStatusTip("Opens a dialog for removing the stops from the training data.");
  QObject::connect( remove_stops_act, &QAction::triggered, [this](){ remove_stops_dialog->lauch(); } );
  remove_stops_dialog = new RemoveStopsDialog(this);
  QObject::connect( remove_stops_dialog, &RemoveStopsDialog::removeStopsSignal, this, [this]()
  { showMsg( ctrl_->removeStopsFromDemo( remove_stops_dialog->getVelTrimThres() ) ); });

  undo_trim_data_act = new QAction("Undo trimming", this);
  undo_trim_data_act->setStatusTip("Undos the trimming of the training data.");
  QObject::connect( undo_trim_data_act, &QAction::triggered, this, [this](){ showMsg(ctrl_->undoTrimDemoData()); } );

  clear_train_data_act = new QAction("Clear train data", this);
  clear_train_data_act->setStatusTip("Clears the training data.");
  QObject::connect( clear_train_data_act, &QAction::triggered, this, [this]()
  {
    ctrl_->clearDemoData();
    updateGUIonTrainData();
    showInfoMsg("All training data were cleared!\n");
  });

  plot_train_data_act = new QAction("demo data", this);
  plot_train_data_act->setStatusTip("Plots the training data.");
  QObject::connect( plot_train_data_act, &QAction::triggered, this, [this](){ std::thread([this](){ ctrl_->plotTrainData(); } ).detach(); });

  plot_demo_sim_data_act = new QAction("demo vs model data", this);
  plot_demo_sim_data_act->setStatusTip("Simulates the model and plots the data.");
  QObject::connect( plot_demo_sim_data_act, &QAction::triggered, this, [this](){ std::thread([this](){ ctrl_->plotModelSimData(); } ).detach(); });

  close_all_plots_act = new QAction("close all", this);
  close_all_plots_act->setStatusTip("Closes all figures.");
  QObject::connect( close_all_plots_act, &QAction::triggered, this, [this]()
  { std::thread([this](){ pl_::closeAll(); } ).detach(); });

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

  view_target_pose_act = new QAction("target pose", this);
  view_target_pose_act->setStatusTip("Display the target pose frame in rviz.");
  view_target_pose_act->setCheckable(true);
  view_target_pose_act->setChecked(false);
  QObject::connect( view_target_pose_act, &QAction::triggered, this, [this]()
  {
    static bool is_view_target_pose_act_checked = false;
    is_view_target_pose_act_checked = !is_view_target_pose_act_checked;
    view_target_pose_act->setChecked(is_view_target_pose_act_checked);
    ctrl_->viewTargetPose(is_view_target_pose_act_checked);
  });

  view_progress_dialog = new gui_::ViewProgressDialog(std::bind(&TrainController::getProgress, this->ctrl_), this);
  view_progress_dialog->setWindowTitle(QString(getNodeNameID().c_str()) + ":Execution progress");
  view_exec_progress_act = new QAction("Exec progress", this);
  view_exec_progress_act->setStatusTip("Open a dialog displaying the execution progress.");
  QObject::connect( view_exec_progress_act, &QAction::triggered, this, [this]()
  { view_progress_dialog->launch(); });
  QObject::connect(this, &TrainWin::execStartedSignal, view_progress_dialog, &gui_::ViewProgressDialog::startProgressBarUpdate);
  QObject::connect(this, &TrainWin::execStopedSignal, view_progress_dialog, &gui_::ViewProgressDialog::stopProgressBarUpdate);
  // QObject::connect(this, &TrainWin::execDirChangedSignal, this, [this](int dir)
  // {
  //   if (dir > 0) view_progress_dialog->setOperationType(ViewProgressDialog::FORWARD_OP);
  //   else view_progress_dialog->setOperationType(ViewProgressDialog::REVERSE_OP);
  // });

  exec_train_traj_act = new QAction("Execute train trajectory", this);
  exec_train_traj_act->setStatusTip("Executes the training trajectory.");
  QObject::connect( exec_train_traj_act, &QAction::triggered, this, [this](){ std::thread([this]()
  {
    view_progress_dialog->startProgressBarUpdate();
    ctrl_->executeTrainTrajectory(); } ).detach();
  });

  exec_rev_train_traj_act = new QAction("Execute reverse train trajectory", this);
  exec_rev_train_traj_act->setStatusTip("Executes the reverse training trajectory.");
  QObject::connect( exec_rev_train_traj_act, &QAction::triggered, this, [this](){ std::thread([this]()
  {
    view_progress_dialog->startProgressBarUpdate();
    ctrl_->executeTrainTrajectory(true); } ).detach();
  });

  QObject::connect(this, &TrainWin::ctrl_execModelFinishedSignal, this, [this](const ExecResultMsg &msg)
  {
    view_progress_dialog->stopProgressBarUpdate();
    if (!msg.empty()) showMsg(msg);
  });

}

void TrainWin::createMenu()
{
  QMenuBar *menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  QMenu *file_menu = menu_bar->addMenu(tr("&File"));
    file_menu->addAction(load_train_data_act);
    file_menu->addAction(load_model_act);
    file_menu->addAction(load_params_act);
    file_menu->addSeparator();
    file_menu->addAction(save_train_data_act);
    file_menu->addAction(save_model_act);

  QMenu *edit_menu = menu_bar->addMenu(tr("&Edit"));
    edit_menu->addAction(set_target_pose_act);
    edit_menu->addAction(set_current_pose_as_target_act);
    edit_menu->addAction(set_start_pose_from_train_data_act);
    edit_menu->addAction(set_target_pose_from_model_data_act);
    edit_menu->addSeparator();
    edit_menu->addAction(trim_data_act);
    edit_menu->addAction(undo_trim_data_act);
    edit_menu->addAction(remove_stops_act);
    edit_menu->addAction(clear_train_data_act);

  QMenu *view_menu = menu_bar->addMenu(tr("&View"));
    QMenu *plot_menu = view_menu->addMenu(tr("&Plot"));
      plot_menu->addAction(plot_train_data_act);
      plot_menu->addAction(plot_demo_sim_data_act);
      plot_menu->addAction(close_all_plots_act);
    QMenu *rviz_menu = view_menu->addMenu(tr("&Rviz"));
      rviz_menu->addAction(view_learned_path_act);
      rviz_menu->addAction(view_target_pose_act);
    view_menu->addAction(view_exec_progress_act);

  QMenu *cmd_menu = menu_bar->addMenu(tr("&Command"));
    cmd_menu->addAction(exec_train_traj_act);
    cmd_menu->addAction(exec_rev_train_traj_act);
}

QFrame *TrainWin::createImpedanceParamsFrame()
{
  damp_pos_slider = new ParamSlider(0, 200, "Dp");
  QObject::connect( damp_pos_slider, &ParamSlider::valueChanged, this, [this]()
  { ctrl_->model->setPosDamping(damp_pos_slider->getSliderPos()); });
  //emit damp_pos_slider->valueChanged();

  stiff_pos_slider = new ParamSlider(0, 1000, "Kp");
  QObject::connect( stiff_pos_slider, &ParamSlider::valueChanged, this, [this]()
  { ctrl_->model->setPosStiffness(stiff_pos_slider->getSliderPos()); });
  //emit stiff_pos_slider->valueChanged();

  damp_orient_slider = new ParamSlider(0, 10, "Do");
  QObject::connect( damp_orient_slider, &ParamSlider::valueChanged, this, [this]()
  { ctrl_->model->setOrientDamping(damp_orient_slider->getSliderPos()); });
  //emit damp_orient_slider->valueChanged();

  stiff_orient_slider = new ParamSlider(0, 20, "Ko");
  QObject::connect( stiff_orient_slider, &ParamSlider::valueChanged, this, [this]()
  { ctrl_->model->setOrientStiffness(stiff_orient_slider->getSliderPos()); });
  //emit stiff_orient_slider->valueChanged();

  modify_path_chkbox = new QCheckBox("allow path modifications");
  modify_path_chkbox->setChecked(false);
  modify_path_chkbox->setFont(font3);
  QObject::connect( modify_path_chkbox, &QCheckBox::stateChanged, this, [this]()
  {
    bool set = modify_path_chkbox->isChecked();
    damp_pos_slider->setVisible(set);
    stiff_pos_slider->setVisible(set);
    damp_orient_slider->setVisible(set);
    stiff_orient_slider->setVisible(set);
  });
  emit modify_path_chkbox->stateChanged(Qt::Unchecked);

  QVBoxLayout *imp_layout = new QVBoxLayout;
  imp_layout->addWidget(modify_path_chkbox);
  imp_layout->addWidget(damp_pos_slider);
  imp_layout->addWidget(stiff_pos_slider);
  imp_layout->addWidget(damp_orient_slider);
  imp_layout->addWidget(stiff_orient_slider);

  QFrame *imp_frame = new QFrame;
  imp_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  imp_frame->setLineWidth(1);
  imp_frame->setLayout(imp_layout);

  return imp_frame;
}


QFrame *TrainWin::createPhaseCtrlFrame()
{
  fv_scaling_ = new gui_::LabelLineEdit("fv scaling:","1");
  fv_scaling_->setFont(font3);
  QObject::connect( fv_scaling_, &gui_::LabelLineEdit::editingFinished, this, [this]()
  { ctrl_->setFvScaling(fv_scaling_->getValue()); });
  emit fv_scaling_->editingFinished();

  fv_filt_ = new gui_::LabelLineEdit("fv filter:","0.01");
  fv_filt_->setFont(font3);
  QObject::connect( fv_filt_, &gui_::LabelLineEdit::editingFinished, this, [this]()
  { ctrl_->setFvFilter(fv_filt_->getValue()); });
  emit fv_filt_->editingFinished();

  x_damp_ = new gui_::LabelLineEdit("Dx:","50");
  x_damp_->setFont(font3);
  QObject::connect( x_damp_, &gui_::LabelLineEdit::editingFinished, this, [this]()
  { ctrl_->setPhaseDamping(x_damp_->getValue()); });
  emit x_damp_->editingFinished();

  QHBoxLayout *phaseCtrl_params_layout = new QHBoxLayout;
  phaseCtrl_params_layout->addWidget(x_damp_);
  phaseCtrl_params_layout->addSpacing(15);
  phaseCtrl_params_layout->addWidget(fv_filt_);
  phaseCtrl_params_layout->addSpacing(15);
  phaseCtrl_params_layout->addWidget(fv_scaling_);

  phaseCtrl_params_frame = new QFrame;
  phaseCtrl_params_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  phaseCtrl_params_frame->setLineWidth(1);
  phaseCtrl_params_frame->setLayout(phaseCtrl_params_layout);

  return phaseCtrl_params_frame;
}

QFrame *TrainWin::createParamsFrame()
{
  train_btn = new QPushButton("train");
  train_btn->setFont(font1);
  train_btn->setEnabled(ctrl_->isTrainData());
  QObject::connect( train_btn, &QPushButton::pressed, this, [this]()
  {
    ExecResultMsg msg = ctrl_->trainModel();
    updateGUIonModel();
    showMsg(msg);
  });

  QLabel *params_lb = new QLabel("Train Params");
  params_lb->setFont(font1);

  QLabel *Nkernels_lb = new QLabel("N_kernels:");
  Nkernels_lb->setFont(font1);
  Nkernels_le = new QLineEdit();
  Nkernels_le->setAlignment(Qt::AlignCenter);
  Nkernels_le->setFont(font2);
  Nkernels_le->setText("20");
  QObject::connect( Nkernels_le, &QLineEdit::editingFinished, this, [this]()
  { ctrl_->setNkernels(Nkernels_le->text().toULong()); });
  emit Nkernels_le->editingFinished();


  QLabel *train_meth_lb = new QLabel("train_method:");
  train_meth_lb->setFont(font1);

  train_meth_cmbox = new QComboBox();
  train_meth_cmbox->setFont(font2);
  train_meth_cmbox->addItem("LS");
  train_meth_cmbox->addItem("LWR");
  QObject::connect( train_meth_cmbox, &QComboBox::currentTextChanged, this, [this]()
  { ctrl_->setTrainMethod(train_meth_cmbox->currentText().toStdString()); });
  train_meth_cmbox->setCurrentText("LS"); // this should automatically emit the signal below, but for some reason it doesn't...
  emit train_meth_cmbox->currentTextChanged("");

  QLabel *model_type_lb = new QLabel("model type:");
  model_type_lb->setFont(font1);

  model_type_cmbox = new QComboBox();
  model_type_cmbox->setFont(font2);
  model_type_cmbox->addItem("std");
  model_type_cmbox->addItem("target");
  // do this before signal connection to avoid overwritting the model
  if (ctrl_->model->getType() == Model::STD) model_type_cmbox->setCurrentText("std");
  else if (ctrl_->model->getType() == Model::TARGET) model_type_cmbox->setCurrentText("target");
  QObject::connect( model_type_cmbox, &QComboBox::currentTextChanged, this, [this]()
  {
    ctrl_->setModelType(model_type_cmbox->currentText().toStdString());
    bool is_model_trained = ctrl_->isModelTrained();
    phase2_rbtn->setEnabled(!is_train_on && is_model_trained);
    phase3_rbtn->setEnabled(!is_train_on && is_model_trained);
    phase4_rbtn->setEnabled(!is_train_on && is_model_trained);
    view_learned_path_act->setEnabled(is_model_trained);
    save_model_act->setEnabled(is_model_trained);
    plot_demo_sim_data_act->setEnabled(is_model_trained && ctrl_->isTrainData());
    set_idle_phase();
  });
  // emit model_type_cmbox->currentTextChanged(""); Dont do this to avoid overwritting the model

  QVBoxLayout *params_layout = new QVBoxLayout;
  params_layout->addStretch(0);
  QGridLayout *temp_layout = new QGridLayout;
    temp_layout->addWidget(params_lb, 0,0, 1,2, Qt::AlignCenter);
    temp_layout->addWidget(Nkernels_lb, 1,0);
    temp_layout->addWidget(Nkernels_le, 1,1);
    temp_layout->addWidget(train_meth_lb, 2,0);
    temp_layout->addWidget(train_meth_cmbox, 2,1);
    temp_layout->addWidget(model_type_lb, 3,0);
    temp_layout->addWidget(model_type_cmbox, 3,1);
    temp_layout->addWidget(train_btn, 4,0, 1,2, Qt::AlignCenter);
  params_layout->addLayout(temp_layout);
  params_layout->addStretch(0);

  QFrame *frame = new QFrame;
  frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  frame->setLineWidth(4);
  frame->setLayout(params_layout);

  return frame;
}

QFrame *TrainWin::createEqulizeVelocityProfileFrame()
{
  QFont font1 = QFont("Ubuntu", 17, QFont::DemiBold);
  QFont font2 = QFont("Ubuntu", 15, QFont::DemiBold);

  QLabel *title_lb = new QLabel("Velocity profile equalization");
  title_lb->setFont(font1);
  title_lb->setStyleSheet("QLabel { color: rgb(0, 0, 255); background-color: rgb(220, 220, 220, 100); }");

  eq_vel_prof_btn = new QPushButton("equalize velocity profile");
  eq_vel_prof_btn->setFont(font2);
  eq_vel_prof_btn->setEnabled(ctrl_->isModelTrained());
  QObject::connect( eq_vel_prof_btn, &QPushButton::pressed, this, [this]()
  { showMsg( ctrl_->equalizeVelocityProfile() ); });

  plot_vel_prof_btn = new QPushButton("plot velocity profile");
  plot_vel_prof_btn->setFont(font2);
  plot_vel_prof_btn->setEnabled(ctrl_->isModelTrained());
  QObject::connect( plot_vel_prof_btn, &QPushButton::pressed, this, [this]()
  { std::thread([this](){ ctrl_->plotVelocityProfile(); } ).detach(); });

  QLabel *nom_vel_lb = new QLabel("nominal velocity:");
  nom_vel_lb->setFont(font2);
  nom_vel_le = new QLineEdit();
  nom_vel_le->setAlignment(Qt::AlignCenter);
  nom_vel_le->setFont(font2);
  nom_vel_le->setText("0.1");
  QObject::connect( nom_vel_le, &QLineEdit::editingFinished, this, [this]()
  { ctrl_->setEqualizationNominalVelocity(nom_vel_le->text().toDouble()); });
  emit nom_vel_le->editingFinished();

  QLabel *eq_iters_lb = new QLabel("iterations:");
  eq_iters_lb->setFont(font2);
  eq_iters_le = new QLineEdit();
  eq_iters_le->setAlignment(Qt::AlignCenter);
  eq_iters_le->setFont(font2);
  eq_iters_le->setText("4");
  QObject::connect( eq_iters_le, &QLineEdit::editingFinished, this, [this]()
  { ctrl_->setEqualizationIterations(eq_iters_le->text().toULong()); });
  emit eq_iters_le->editingFinished();


  QVBoxLayout *params_layout = new QVBoxLayout;
  params_layout->addStretch(0);
  params_layout->addWidget(title_lb);
  QGridLayout *temp_layout = new QGridLayout;
    temp_layout->addWidget(nom_vel_lb, 0,0);
    temp_layout->addWidget(nom_vel_le, 0,1);
    temp_layout->addWidget(eq_iters_lb, 1,0);
    temp_layout->addWidget(eq_iters_le, 1,1);
  params_layout->addLayout(temp_layout);
  params_layout->addWidget(eq_vel_prof_btn);
  params_layout->addWidget(plot_vel_prof_btn);
  params_layout->addStretch(0);

  QFrame *frame = new QFrame;
  frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  frame->setLineWidth(4);
  frame->setLayout(params_layout);

  return frame;
}


QFrame *TrainWin::createTrainFrame()
{
  phase1_rbtn = new QRadioButton("Phase 1: Path teaching");
  phase1_rbtn->setFont(font1);
  phase2_rbtn = new QRadioButton("Phase 2: Velocity teaching");
  phase2_rbtn->setFont(font1);
  phase2_rbtn->setEnabled(ctrl_->isModelTrained());
  phase3_rbtn = new QRadioButton("Phase 3: Adapt learned trajectory");
  phase3_rbtn->setFont(font1);
  phase3_rbtn->setEnabled(ctrl_->isModelTrained());
  phase4_rbtn = new QRadioButton("Phase 4: Execute learned trajectory");
  phase4_rbtn->setFont(font1);
  phase4_rbtn->setEnabled(ctrl_->isModelTrained());
  idle_rbtn = new QRadioButton("Idle");
  idle_rbtn->setEnabled(true);
  idle_rbtn->setVisible(false);
  idle_rbtn->setChecked(true);
  phase_rbtn = new QButtonGroup();
  phase_rbtn->addButton(phase1_rbtn, Train_Phase1);
  phase_rbtn->addButton(phase2_rbtn, Train_Phase2);
  phase_rbtn->addButton(phase3_rbtn, Train_Phase3);
  phase_rbtn->addButton(phase4_rbtn, Train_Phase4);
  phase_rbtn->addButton(idle_rbtn, Train_IDLE_phase);
  QObject::connect( phase_rbtn, SIGNAL(buttonClicked(QAbstractButton *)), this, SLOT(phaseChanged()) );
  set_idle_phase = [this]()
  {
    idle_rbtn->setChecked(true);
    phase_rbtn->buttonClicked(idle_rbtn);
  };

  QLabel *method_lb = new QLabel("method:");
  method_lb->setFont(font1);
  ph1_meth_cmbox = new QComboBox();
  ph1_meth_cmbox->setFont(font2);
  ph1_meth_cmbox->addItem("FREEDRIVE");
  ph1_meth_cmbox->addItem("ADMITTANCE");
  QObject::connect( ph1_meth_cmbox, &QComboBox::currentTextChanged, this, [this]()
  {
    std::string ph1_method = ph1_meth_cmbox->currentText().toStdString();
    if (ph1_method.compare("FREEDRIVE") == 0) path_teach_method = PathTeach_FREEDRIVE;
    else if (ph1_method.compare("ADMITTANCE") == 0) path_teach_method = PathTeach_ADMITTANCE;

    if (getTeachingPhase() == Train_Phase1)
    {
      ctrl_->setPathTeachMethod(path_teach_method);
      showMsg(ExecResultMsg(ExecResultMsg::INFO,"The robot is ready!"));
    }

  });
  ph1_meth_cmbox->setCurrentText("FREEDRIVE"); // this should automatically emit the signal below, but for some reason it doesn't...
  emit ph1_meth_cmbox->currentTextChanged("");

  slider_rbtn = new QRadioButton("slider");
  slider_rbtn->setFont(font2);
  phri_rbtn = new QRadioButton("pHRI");
  phri_rbtn->setFont(font2);
  phri_rbtn->setChecked(true);
  vel_teach_method_rbtn = new QButtonGroup();
  vel_teach_method_rbtn->addButton(phri_rbtn, VelTeach_pHRI);
  vel_teach_method_rbtn->addButton(slider_rbtn, VelTeach_SLIDER);
  QObject::connect( vel_teach_method_rbtn, SIGNAL(buttonClicked(QAbstractButton *)), this, SLOT(velTeachMethodChanged()) );

  start_btn = new QPushButton("start");
  start_btn->setFont(font1);
  QObject::connect( start_btn, &QPushButton::pressed, this, [this](){ startBtnPressed(); }) ;

  stop_btn = new QPushButton("stop");
  stop_btn->setFont(font1);
  stop_btn->setEnabled(false);
  QObject::connect( stop_btn, &QPushButton::pressed, this, [this](){ stopBtnPressed(); }) ;
  QObject::connect( this, &TrainWin::stopTrainingSignal, this, [this](const ExecResultMsg &msg)
  {
    is_train_on = false;
    updateGUIonStartStopBtn();
    showMsg(msg);
  });

  train_slider = new gui_::PhaseCtrlSlider(-2,2);
  train_slider->setFrameStyle(QFrame::Box | QFrame::Raised);
  train_slider->setLineWidth(2);
  train_slider->setVisible(false);

  logging_chkbox = new QCheckBox("Enable logging");
  logging_chkbox->setChecked(false);
  logging_chkbox->setFont(font3);
  QObject::connect( logging_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->enable_logging.set(logging_chkbox->isChecked()); });

  autotrim_train_data_chkbox = new QCheckBox("autotrim train dat");
  autotrim_train_data_chkbox->setChecked(false);
  autotrim_train_data_chkbox->setFont(font3);
  QObject::connect( autotrim_train_data_chkbox, &QCheckBox::stateChanged, this, [this]()
  { 
    ctrl_->autotrim_train_data.set(autotrim_train_data_chkbox->isChecked());
    move_thres_le->setVisible(autotrim_train_data_chkbox->isChecked());
    move_thres_units_lb->setVisible(autotrim_train_data_chkbox->isChecked());
  });
  move_thres_le = new QLineEdit("1");
  move_thres_le->setAlignment(Qt::AlignCenter);
  move_thres_le->setFont(font2);
  move_thres_le->setVisible(false);
  move_thres_le->setMaximumWidth(55);
  move_thres_units_lb = new QLabel("mm");
  move_thres_units_lb->setFont(font2);
  move_thres_units_lb->setVisible(false);
  move_thres_units_lb->setMaximumWidth(40);
  QHBoxLayout *autotrim_train_layout = new QHBoxLayout;
  autotrim_train_layout->addWidget(autotrim_train_data_chkbox);
  autotrim_train_layout->addWidget(move_thres_le);
  autotrim_train_layout->addWidget(move_thres_units_lb);
  autotrim_train_layout->addStretch(0);

  phase_ff_chkbox = new QCheckBox("Phase feedforward");
  phase_ff_chkbox->setChecked(false);
  phase_ff_chkbox->setFont(font3);
  QObject::connect( phase_ff_chkbox, &QCheckBox::stateChanged, this, [this]()
  { 
    ctrl_->phase_ff.set(phase_ff_chkbox->isChecked()); 
    phase_ff_le->setVisible(phase_ff_chkbox->isChecked());
  });
  phase_ff_le = new QLineEdit("0");
  phase_ff_le->setAlignment(Qt::AlignCenter);
  phase_ff_le->setFont(font2);
  phase_ff_le->setVisible(false);
  phase_ff_le->setMaximumWidth(60);
  QHBoxLayout *phase_ff_layout = new QHBoxLayout;
  phase_ff_layout->addWidget(phase_ff_chkbox);
  phase_ff_layout->addWidget(phase_ff_le);
  phase_ff_layout->addStretch(0);

  online_adapt_chkbox = new QCheckBox("Online adaptation");
  online_adapt_chkbox->setChecked(false);
  online_adapt_chkbox->setVisible(false);
  online_adapt_chkbox->setFont(font3);
  QObject::connect( online_adapt_chkbox, &QCheckBox::stateChanged, this, [this]()
  { ctrl_->enable_online_adaptation.set(online_adapt_chkbox->isChecked()); });

  display_ref_frame_chkbox = new QCheckBox("Display ref frame");
  display_ref_frame_chkbox->setChecked(false);
  display_ref_frame_chkbox->setFont(font3);
  QObject::connect( display_ref_frame_chkbox, &QCheckBox::stateChanged, this, [this]()
  {
    if ( display_ref_frame_chkbox->isChecked() ) ctrl_->ref_pose_pub->start(50);
    else ctrl_->ref_pose_pub->stop();
  });

  phaseCtrl_params_frame = createPhaseCtrlFrame();

  phase4_frame = new QFrame;
  {
    QVBoxLayout *phase4_layout = new QVBoxLayout;
      QHBoxLayout *tau_layout = new QHBoxLayout;
        QLabel *tau_label = new QLabel("tau:");
        tau_label->setFont(font2);
        tau_label->setAlignment(Qt::AlignCenter);
        QLineEdit *tau_le = new QLineEdit;
        tau_le->setText("10");
        tau_le->setFont(font2);
        tau_le->setAlignment(Qt::AlignCenter);
        tau_le->setMaximumWidth(80);
        QObject::connect( tau_le, &QLineEdit::editingFinished, this, [this, tau_le]() { phase4.tau = tau_le->text().toDouble(); });
        emit tau_le->editingFinished();
        tau_layout->addWidget(tau_label);
        tau_layout->addWidget(tau_le);
        tau_layout->addStretch(0);
    phase4_layout->addLayout(tau_layout);
      QCheckBox *reverse_chkbox = new QCheckBox("reverse");
      reverse_chkbox->setChecked(false);
      reverse_chkbox->setFont(font2);
      QObject::connect( reverse_chkbox, &QCheckBox::stateChanged, this, [this, reverse_chkbox]() { phase4.reverse = reverse_chkbox->isChecked(); });
    phase4_layout->addWidget(reverse_chkbox);
    phase4_layout->addStretch(0);

    phase4_frame->setLayout(phase4_layout);
  }
  

  //imp_params_frame = createImpedanceParamsFrame();

  // =============  Layouts  ================

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  QVBoxLayout *phase_layout = new QVBoxLayout;
  { // widgets for phase-1
    phase_layout->addWidget(phase1_rbtn, 0);
    QHBoxLayout *ph1_method_layout = new QHBoxLayout;
      ph1_method_layout->addWidget(method_lb);
      ph1_method_layout->addWidget(ph1_meth_cmbox);
    phase_layout->addLayout(ph1_method_layout, 1);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  { // widgets for phase-2
    phase_layout->addWidget(phase2_rbtn, 2);
    QHBoxLayout *ph2_method_layout = new QHBoxLayout;
      ph2_method_layout->insertSpacing(0,30);
      QGridLayout *vel_teach_method_layout = new QGridLayout;
        vel_teach_method_layout->addWidget(method_lb, 0,0);
        vel_teach_method_layout->addWidget(slider_rbtn, 0,1);
        vel_teach_method_layout->addWidget(phri_rbtn, 1,1);
      ph2_method_layout->addLayout(vel_teach_method_layout);
      ph2_method_layout->addStretch(0);
    // -------------------------------------------
    phase_layout->addLayout(ph2_method_layout, 3);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  { // widgets for phase-3
    QHBoxLayout *ph3_method_layout = new QHBoxLayout;
    ph3_method_layout->addWidget(phase3_rbtn);
    phase_layout->addLayout(ph3_method_layout, 5);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  { // widgets for phase-4
    QHBoxLayout *ph4_method_layout = new QHBoxLayout;
    ph4_method_layout->addWidget(phase4_rbtn);
    phase_layout->addLayout(ph4_method_layout, 6);
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  QHBoxLayout *start_stop_btns_layout = new QHBoxLayout;
    start_stop_btns_layout->addWidget(start_btn);
    start_stop_btns_layout->addWidget(stop_btn);
    //sst_btns_layout->insertSpacing(2,40);
    start_stop_btns_layout->addStretch(0);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  QVBoxLayout *train_layout = new QVBoxLayout;
    train_layout->addLayout(phase_layout);
    train_layout->addWidget(phaseCtrl_params_frame);
    train_layout->addWidget(train_slider);
    train_layout->addWidget(phase4_frame);
    //train_layout->addWidget(imp_params_frame);
    train_layout->addWidget(logging_chkbox);
    train_layout->addLayout(autotrim_train_layout);
    train_layout->addLayout(phase_ff_layout);
    train_layout->addWidget(online_adapt_chkbox);
    train_layout->addWidget(display_ref_frame_chkbox);
    train_layout->addLayout(start_stop_btns_layout);
    train_layout->addStretch(0);

  QFrame *train_frame = new QFrame;
    train_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
    train_frame->setLineWidth(4);
    train_frame->setLayout(train_layout);

  return train_frame;
}

TrainWin::~TrainWin()
{
    stop();
}

void TrainWin::launch()
{
  if (!run)
  {
    run = true;
    idle_rbtn->setChecked(true);
    phaseChanged();
    this->show();
  }
}


void TrainWin::stop()
{
  if (run) run = false;
  this->hide();
}

void TrainWin::phaseChanged()
{
  phase = static_cast<TrainPhase>(phase_rbtn->checkedId());

  bool is_phase1 = phase == Train_Phase1;
  bool is_phase2 = phase == Train_Phase2;
  bool is_phase3 = phase == Train_Phase3;
  bool is_phase4 = phase == Train_Phase4;

  ph1_meth_cmbox->setEnabled(is_phase1);

  slider_rbtn->setEnabled(is_phase2);
  phri_rbtn->setEnabled(is_phase2);
  phaseCtrl_params_frame->setVisible(is_phase2 || is_phase3);
  train_slider->setVisible(is_phase2 && vel_teach_method==VelTeach_SLIDER);

  online_adapt_chkbox->setVisible(is_phase3);

  phase4_frame->setVisible(is_phase4);

  display_ref_frame_chkbox->setVisible(is_phase2 || is_phase3);
  view_exec_progress_act->setEnabled(is_phase2 || is_phase3);
  //imp_params_frame->setVisible(set);

  updateGUIonRadioBtn();

  if (phase != Train_IDLE_phase)
  {
    ctrl_->trainPhaseChanged(phase);
    showMsg(ExecResultMsg(ExecResultMsg::INFO,"The robot is ready!"));
  }

}

void TrainWin::velTeachMethodChanged()
{
  vel_teach_method = static_cast<VelTeachMethod>(vel_teach_method_rbtn->checkedId());
  train_slider->setVisible(vel_teach_method==VelTeach_SLIDER);
}

void TrainWin::startBtnPressed()
{
  if (is_train_on) return;
  is_train_on = true;
  updateGUIonStartStopBtn();
  if (phase == Train_Phase2 || phase == Train_Phase3) emit execStartedSignal();
  std::thread([this](){ ctrl_->startDemoRec(); }).detach();
}

void TrainWin::stopBtnPressed()
{
  if (!is_train_on) return;
  is_train_on = false;
  updateGUIonStartStopBtn();
  if (phase == Train_Phase2) emit execStopedSignal();
  showMsg(ctrl_->stopDemoRec());
}

void TrainWin::updateGUIonStartStopBtn()
{
  if (is_train_on) start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  else start_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  bool is_model_trained = ctrl_->isModelTrained();

  stop_btn->setEnabled(is_train_on);
  phase1_rbtn->setEnabled(!is_train_on);
  phase2_rbtn->setEnabled(!is_train_on && is_model_trained);
  phase3_rbtn->setEnabled(!is_train_on && is_model_trained);
  phase4_rbtn->setEnabled(!is_train_on && is_model_trained);
  slider_rbtn->setEnabled(!is_train_on && phase==2);
  phri_rbtn->setEnabled(!is_train_on && phase==2);
  params_frame->setEnabled(!is_train_on);
  train_btn->setEnabled(!is_train_on && ctrl_->isTrainData());

  updateGUIonTrainData();
}

void TrainWin::updateGUIonTrainData()
{
  bool set = ctrl_->isTrainData();

  clear_train_data_act->setEnabled(set);
  train_btn->setEnabled(set);
  save_train_data_act->setEnabled(set);
  set_start_pose_from_train_data_act->setEnabled(set);
  trim_data_act->setEnabled(set);
  undo_trim_data_act->setEnabled(set);
  plot_train_data_act->setEnabled(set);
  plot_demo_sim_data_act->setEnabled(set && ctrl_->isModelTrained());
}

void TrainWin::updateGUIonModel(bool set)
{
  bool is_model_trained = ctrl_->isModelTrained();

  phase2_rbtn->setEnabled(is_model_trained && set);
  save_model_act->setEnabled(is_model_trained && set);

  phase3_rbtn->setEnabled(is_model_trained && set);
  phase4_rbtn->setEnabled(is_model_trained && set);

  plot_demo_sim_data_act->setEnabled(is_model_trained && ctrl_->isTrainData());

  view_learned_path_act->setEnabled(is_model_trained);

  eq_vel_prof_btn->setEnabled(is_model_trained);
  plot_vel_prof_btn->setEnabled(is_model_trained);

  set_target_pose_from_model_data_act->setEnabled(is_model_trained && set);

  Model::Type type = ctrl_->model->getType();
  model_type_cmbox->blockSignals(true);
  if (type == Model::Type::STD) model_type_cmbox->setCurrentText("std");
  else if (type == Model::Type::TARGET) model_type_cmbox->setCurrentText("target");
  model_type_cmbox->blockSignals(false);
}

void TrainWin::updateGUIonRadioBtn()
{
  bool set = !(idle_rbtn->isChecked());

  start_btn->setEnabled(set);
}

void TrainWin::closeEvent(QCloseEvent *event)
{
  stop();
}

ExecResultMsg TrainWin::loadParams(const std::string &path)
{
  try
  {
    io_::XmlParser parser(path);

    double Dx;
    if (parser.getParam("Dx", Dx))
    {
      x_damp_->setValue(Dx);
      emit x_damp_->editingFinished();
    }

    double fv_filt;
    if (parser.getParam("fv_filt", fv_filt))
    {
      fv_filt_->setValue(fv_filt);
      emit fv_filt_->editingFinished();
    }

    double fv_scaling;
    if (parser.getParam("fv_scaling", fv_scaling))
    {
      fv_scaling_->setValue(fv_scaling);
      emit fv_scaling_->editingFinished();
    }

    unsigned N_kernels;
    if (parser.getParam("N_kernels", N_kernels))
    {
      Nkernels_le->setText(QString::number(N_kernels));
      emit Nkernels_le->editingFinished();
    }

    std::string train_method;
    if (parser.getParam("train_method", train_method)) train_meth_cmbox->setCurrentText(train_method.c_str());

    double slider_min_val;
    if (parser.getParam("slider_min_val", slider_min_val)) train_slider->setMinValue(slider_min_val);

    double slider_max_val;
    if (parser.getParam("slider_max_val", slider_max_val)) train_slider->setMaxValue(slider_max_val);

    double slider_reset_rate;
    if (parser.getParam("slider_reset_rate", slider_reset_rate)) train_slider->setResetRate(slider_reset_rate);

    bool slider_auto_reset;
    if (parser.getParam("slider_auto_reset", slider_auto_reset)) train_slider->autoReset(slider_auto_reset);

    bool enable_logging;
    if (parser.getParam("enable_logging", enable_logging))
    {
      logging_chkbox->setChecked(enable_logging);
      emit logging_chkbox->stateChanged(0);
    }

    bool enable_online_adaptation;
    if (parser.getParam("enable_online_adaptation", enable_online_adaptation))
    {
      online_adapt_chkbox->setChecked(enable_online_adaptation);
      emit online_adapt_chkbox->stateChanged(0);
    }

    return ExecResultMsg(ExecResultMsg::INFO, "Training params loaded successfully!");
  }
  catch (const std::exception &e)
  {
    return ExecResultMsg(ExecResultMsg::ERROR, std::string("Error loading training params:\n") + e.what());
  }

}

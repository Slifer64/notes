#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <ros/package.h>

#include <cstring>

#include <global_gui/global_gui.h>

#include <yaml-cpp/yaml.h>

#define GlobalGui_fun_ std::string("[GlobalWindow::") + __func__ + "]: "

GlobalWindow::GlobalWindow(QWidget *parent) : QMainWindow(parent)
{
  // std::vector<std::string> priority_name = {"IdlePriority", "LowestPriority", "LowPriority", "NormalPriority", "HighPriority", "HighestPriority", "TimeCriticalPriority", "InheritPriority"};
  // QThread::Priority priority = QThread::currentThread()->priority();
  // PRINT_INFO_MSG("[GlobalWindow::GlobalWindow]: " + priority_name[priority] + "\n", std::cerr);

  //this->resize(400,350);
  this->setWindowTitle("Global gui");

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  font1 = QFont("Ubuntu", 15, QFont::DemiBold);
  font2 = QFont("Ubuntu", 17, QFont::Bold);

  // --------------------------------------------

  std::vector<ControllerParams> ctrl_params = loadParams();

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  ros_ctrl_pub.resize(ctrl_params.size());
  for (int i=0; i<ctrl_params.size(); i++)
  {
    main_layout->addWidget(createControllerFrame(ctrl_params[i], i));
    ros_ctrl_pub[i] = node.advertise<std_msgs::Int16>(ctrl_params[i].publish_topic, 1);
  }

  // ========= connections =============
  QObject::connect( this, SIGNAL(closeSignal()), this, SLOT(close()) );
}

GlobalWindow::~GlobalWindow()
{}

std::vector<ControllerParams> GlobalWindow::loadParams() const
{
  std::vector<ControllerParams> ctrl_params;

  std::string config_file = ros::package::getPath("global_gui") + "/config/params.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file);

  YAML::Node controllers_node;
  if ( !YAML::getParam(config_node, "controllers", controllers_node) )
    throw std::runtime_error(GlobalGui_fun_ + "Failed to load param 'controllers'");
  if (!controllers_node.IsSequence())
    throw std::runtime_error(GlobalGui_fun_ + "Param 'controllers' must be an array");

  unsigned n_controllers = controllers_node.size();
  ctrl_params.resize(n_controllers);
  for (int i=0; i<n_controllers; i++)
  {
    ControllerParams &ctrl_param = ctrl_params[i];

    const YAML::Node &ctrl_node_i = controllers_node[i];
    std::string i_str = std::to_string(i);

    if ( !YAML::getParam(ctrl_node_i, "name", ctrl_param.name) )
      throw std::runtime_error(GlobalGui_fun_ + "Failed to load param 'controllers["+i_str+"].name'");

    if ( !YAML::getParam(ctrl_node_i, "publish_topic", ctrl_param.publish_topic) )
      throw std::runtime_error(GlobalGui_fun_ + "Failed to load param 'controllers["+i_str+"].publish_topic'");

    YAML::Node actions_node;
    if ( !YAML::getParam(ctrl_node_i, "actions", actions_node) )
      throw std::runtime_error(GlobalGui_fun_ + "Failed to load param 'controllers["+i_str+"].actions'");
    if (!actions_node.IsSequence())
      throw std::runtime_error(GlobalGui_fun_ + "Param 'controllers["+i_str+"].actions' must be an array");

    int n_actions = actions_node.size();
    ctrl_param.resize(n_actions);

    for (int j=0; j<n_actions; j++)
    {
      std::string j_str = std::to_string(j);
      const YAML::Node &act_j_node = actions_node[j];
      if (!act_j_node.IsMap())
        throw std::runtime_error(GlobalGui_fun_ + "Param 'controllers["+i_str+"].actions["+j_str+"]' must be a struct");

      if (!YAML::getParam(act_j_node, "id", ctrl_param.act_id[j]))
        throw std::runtime_error(GlobalGui_fun_ + "Failed to load param 'controllers["+i_str+"].actions["+j_str+"].id'");
      if (!YAML::getParam(act_j_node, "name", ctrl_param.act_name[j]))
        throw std::runtime_error(GlobalGui_fun_ + "Failed to load param 'controllers["+i_str+"].actions["+j_str+"].name'");
      if (!YAML::getParam(act_j_node, "btn_label", ctrl_param.btn_label[j]))
        throw std::runtime_error(GlobalGui_fun_ + "Failed to load param 'controllers["+i_str+"].actions["+j_str+"].btn_label'");
      if (!YAML::getParam(act_j_node, "stylesheet", ctrl_param.stylesheet[j]))
        ctrl_param.stylesheet[j] = "";
    }

  }

  return ctrl_params;
}

QFrame *GlobalWindow::createControllerFrame(const ControllerParams &ctrl_params, int pub_id)
{
  QVBoxLayout *layout = new QVBoxLayout;
  
  QLabel *label = new QLabel(ctrl_params.name.c_str());
  label->setFont(font2);
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("color:rgb(0,0,255); background-color:rgba(210, 210, 210, 100);");
  layout->addWidget(label);

  for (int i=0; i<ctrl_params.numOfActions(); i++)
  {
    QPushButton *btn = new QPushButton(ctrl_params.btn_label[i].c_str());
    btn->setFont(font1);
    if (!ctrl_params.stylesheet[i].empty()) btn->setStyleSheet(ctrl_params.stylesheet[i].c_str());
    int act_id = ctrl_params.act_id[i];
    QObject::connect( btn, &QPushButton::clicked, this, [this, act_id, pub_id]()
    {
      std_msgs::Int16 msg;
      // msg.data = static_cast<int>(MainCtrlAction::SET_MODE_FREEDRIVE);
      msg.data = act_id;
      ros_ctrl_pub[pub_id].publish(msg);
    });
    layout->addWidget(btn);
  }
  layout->addStretch(0);

  // emerg_stop_btn = new QPushButton("Emergency stop");
  // emerg_stop_btn->setFont(font1);
  // emerg_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(210, 210, 210, 100);");
  // QObject::connect( emerg_stop_btn, &QPushButton::pressed, this, [this]()
  // {
  //   std_msgs::Int16 msg;
  //   msg.data = static_cast<int>(MainCtrlAction::TRIGGER_EMERGENCY_STOP);
  //   ros_main_pub.publish(msg);
  // });

  QFrame *frame = new QFrame(central_widget);
  frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  frame->setLineWidth(2);
  frame->setLayout(layout);

  return frame;
}

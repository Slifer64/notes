#include <robotiq_85_gripper/utils/utils.h>
#include <robotiq_85_gripper/utils/xml_parser.h>
#include <robotiq_85_gripper/robotiq_85_gripper.h>


#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <robotiq_85_gripper/utils/httplib.h>

namespace r85_
{

#define R85Gripper_fun_ std::string("[R85Gripper::") + __func__ + "]: "

R85Gripper::R85Gripper(const std::string &param_options)
{
  ros::NodeHandle nh("~");

  XmlRpc::XmlRpcValue options;
  if(!nh.getParam(param_options, options)) throw std::runtime_error(R85Gripper_fun_ + "Failed to load \"" + param_options + "\" param...");

  // {use_sim: true, prefix:"ur5_r85_", http_link:"http://192.168.1.1:80"}

  if (options.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    throw std::runtime_error(R85Gripper_fun_ + "\"" + param_options + "\" must be a struct");

  if (options["use_sim"].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    throw std::runtime_error(R85Gripper_fun_ + "\"" + param_options + ".use_sim\" must be a boolean");
  use_sim = static_cast<bool>(options["use_sim"]);

  if (!use_sim)
  {
    if (options["http_link"].getType() != XmlRpc::XmlRpcValue::TypeString)
      throw std::runtime_error(R85Gripper_fun_ + "\"" + param_options + ".http_link\" must be a string");

    std::string http_link = static_cast<std::string>(options["http_link"]);
    http_client = (void *)( new httplib::Client(http_link.c_str()) );
  }

  std::string prefix = "r85_";
  if (options.hasMember("prefix"))
  {
    if (options["prefix"].getType() != XmlRpc::XmlRpcValue::TypeString)
      throw std::runtime_error(R85Gripper_fun_ + "\"" + param_options + ".prefix\" must be a string");

    prefix = static_cast<std::string>(options["prefix"]);
  }

  joint_pos = 0.4;

  // joint_names = {
  // "gripper_finger1_joint",
  // "gripper_finger2_joint",
  // "gripper_finger1_inner_knuckle_joint",
  // "gripper_finger2_inner_knuckle_joint",
  // "gripper_finger1_finger_tip_joint",
  // "gripper_finger2_finger_tip_joint"};

  joint_names = {
    "_left_finger_joint_1",
    "_left_finger_joint_2",
    "_right_finger_joint_1",
    "_right_finger_joint_2"
  };

  for (int i=0; i<joint_names.size(); i++) joint_names[i] = prefix + joint_names[i];

}

R85Gripper::~R85Gripper()
{
  if (http_client) delete (httplib::Client *)http_client;
}

arma::vec R85Gripper::getJointsPosition() const
{
  double p = getJointPos();

  return arma::vec({p,-0.8*p,p,-0.8*p});
}

bool R85Gripper::moveSim(int width, int force)
{
  double q = getJointPos();
  double qd = 0.01*((60.0-8)/(25-65))*(width - 65) + 0.08;
  int n_steps = 1.0/0.002;
  double dq = (qd - q)/n_steps;
  while (--n_steps)
  {
    q = q + dq;
    setJointPos(q);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  return true;
}

bool R85Gripper::move(int width, int force)
{
  if (use_sim) return moveSim(width, force);

  std::string req_url = "/api/dc/rg2ft/set_width/" + std::to_string(width) + "/" + std::to_string(force) + "";
  auto res = ((httplib::Client *)http_client)->Get(req_url.c_str());
  if (res)
  {
    if (res->status == 200)
    {
      // print_info_msg(res->body + "\n", std::cerr);

      double q = getJointPos();
      double qd = float(width)/100.0;
      int n_steps = 1.0/0.002;
      double dq = (qd - q)/n_steps;
      while (--n_steps)
      {
        q = q + dq;
        setJointPos(q);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }

      return true;
    }
  }
  else
  {
    auto err = res.error();
    print_err_msg("Error closing gripper: " + std::to_string(err) + "\n", std::cerr);
  }
  return false;
}

}; // namespace r85_

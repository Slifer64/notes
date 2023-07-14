#ifndef ROBOTIC_85_GRIPPER_H
#define ROBOTIC_85_GRIPPER_H

#include <vector>
#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <armadillo>

namespace r85_
{

class R85Gripper
{
public:
  /** Constructor. Parses the arguments for the gripper's construction from the parameter server.
   *  @param[in] param_options: the parameter server name, containing the gripper's option as a struct.
   *                            Example format {use_sim: true, prefix: "r85_", http_link: "http://192.168.1.1:80"}
   */ 
  R85Gripper(const std::string &param_options);

  // R85Gripper(bool use_sim, const std::string &prefix="", const std::string http_link="");

  /** Destructor
   */ 
  ~R85Gripper();

  /** Moves the fingers of the gripper until the distance between the fingertips 
   *  or the force perceived by them reaches the input argument limits.
   *  @param[in] width: move the gripper until this width between the fingertips is reached.
   *  @param[in] force: force limit until the grippers stops moving.
   *  @return joint position of the controlled joint.
   */
  bool move(int width, int force);

  /** Returns the joint position of the controlled joint.
   *  @return joint position of the controlled joint.
   */ 
  double getJointPos() const { return joint_pos; }

  /** Returns the joint positions of all joints (controlled and not).
   *  @return joint positions of all joints.
   */ 
  arma::vec getJointsPosition() const;

  /** Returns the names of all joints (controlled and not).
   *  @return names of all joints.
   */
  std::vector<std::string> getJointNames() const { return joint_names; }

  /** Returns the number of joints (controlled and not).
   *  @return number of all joints.
   */
  int getNumOfJoints() const { return joint_names.size(); }

protected:

  bool moveSim(int width, int force);

  void setJointPos(double pos) { joint_pos = pos; }

  bool use_sim;

  double joint_pos;

  std::vector<std::string> joint_names;

  void *http_client;
  // std::unique_ptr<httplib::Client> http_client;

};

}; // namespace r85_

#endif // ROBOTIC_85_GRIPPER_

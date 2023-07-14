#include <iostream>
#include <vector>
#include <cstring>
#include <functional>
#include <thread>
#include <chrono>

#include <armadillo>

#include <ros_lib/move_to_joints_pos_client.h>
#include <ros_lib/move_to_joints_pos_server.h>

using namespace as64_;

class Robot
{
public:
  Robot()
  {

  }

  void moveToJointsPos(const std::vector<double> &joints_pos, double duration=5.3)
  {
    run_ = true;
    double dt = 0.002;
    unsigned long sleep_micro = dt * 1e6;
    t = 0.0;
    if (duration > 0) Tf = duration;
    while (run_)
    {
      if (t >= Tf) break;
      t += dt;
      // std::cerr << "Robot t = " << t << "\n";
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_micro));
    }
  }

  void stopExec()
  {
    run_ = false;
    t=0.0;
  }

  double getExecProgress()
  {
    return t/Tf;
  }

private:

  double Tf=1000.0;
  double t=0.0;
  bool run_;

};

void server_fun()
{
  Robot robot;

  MoveToJointsPosActionServer server("move_to_joints_pos",
              std::bind(&Robot::moveToJointsPos, &robot, std::placeholders::_1, std::placeholders::_2), 
              std::bind(&Robot::stopExec, &robot), 
              std::bind(&Robot::getExecProgress, &robot));

  ros::spin();
}


void client_fun()
{
  MoveToJointsPosActionClient client("move_to_joints_pos");

  std::vector<double> joints_pos = {1.2, 3.6, 4.5};
  client.sendGoal(joints_pos);

  double progress = client.getProgress();
  while (progress < 1)
  {
    std::cout << "progress: " << progress*100 << " %\n";
    client.waitFor(0.5);
    progress = client.getProgress();

    if (!client.waitFor(2.5))
    {
      std::cerr << "Time out! Cancelling...\n";
      client.cancel();
      break;
    }
  }
  std::cout << "*** progress: 100 % ***\n";

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_joints_pos_action_test");

  std::thread server_thr = std::thread([](){ server_fun(); });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::thread client_thr = std::thread([](){ client_fun(); });

  server_thr.join();
  client_thr.join();

  return 0;
}
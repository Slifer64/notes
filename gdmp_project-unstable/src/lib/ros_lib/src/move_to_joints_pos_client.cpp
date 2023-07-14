#include <ros_lib/move_to_joints_pos_client.h>

namespace as64_
{

MoveToJointsPosActionClient::MoveToJointsPosActionClient(std::string name, bool spin_thread, bool wait_for_server) : ac(name, spin_thread)
{
    if (wait_for_server) 
    {
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac.waitForServer(); //will wait for infinite time
    }
}

MoveToJointsPosActionClient::~MoveToJointsPosActionClient(void)
{}

void MoveToJointsPosActionClient::sendGoal(const std::vector<double> &joints_pos)
{
    sendGoal(joints_pos, -1);
}

void MoveToJointsPosActionClient::sendGoal(const std::vector<double> &joints_pos, double duration)
{
    // ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    ros_lib::MoveToJointsPosGoal goal;
    goal.target_joints_pos = joints_pos;
    goal.duration = duration;
    // ac.sendGoal(goal);
    ac.sendGoal(goal,
                boost::bind(&MoveToJointsPosActionClient::doneCb, this, _1, _2),
                boost::bind(&MoveToJointsPosActionClient::activeCb, this),
                boost::bind(&MoveToJointsPosActionClient::feedbackCb, this, _1));
}

void MoveToJointsPosActionClient::cancel()
{
    // ROS_INFO("Cancelling goal...");
    ac.cancelGoal();
}

bool MoveToJointsPosActionClient::waitFor(double t_sec)
{
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(t_sec));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    return finished_before_timeout;
}

} // namespace as64_

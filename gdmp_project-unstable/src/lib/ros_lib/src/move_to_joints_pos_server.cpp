#include <ros_lib/move_to_joints_pos_server.h>

namespace as64_
{
  
MoveToJointsPosActionServer::MoveToJointsPosActionServer(std::string name,
            std::function<void(const std::vector<double> &, double)> run_exec, 
            std::function<void()> stop_exec, 
            std::function<ros_lib::MoveToJointsPosFeedback()> get_feedback,
            std::function<ros_lib::MoveToJointsPosResult()> get_result) :
    as_(nh_, name, boost::bind(&MoveToJointsPosActionServer::executeCB, this, _1), false),
    action_name_(name)
{
    this->run_exec_fun = run_exec;
    this->stop_exec_fun = stop_exec;
    this->get_feedback_fun = get_feedback;
    this->get_result_fun = get_result;
    as_.start();
}

MoveToJointsPosActionServer::~MoveToJointsPosActionServer(void)
{}

void MoveToJointsPosActionServer::executeCB(const ros_lib::MoveToJointsPosGoalConstPtr &goal)
{
    // helper variables
    // ros::Rate r(1);
    bool success = true;

    feedback_.progress = 0.0;

    // publish info to the console for the user
    // ROS_INFO("%s: Executing, creating MoveToJointsPos sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    std::thread([this, goal](){ this->run_exec_fun(goal->target_joints_pos, goal->duration); }).detach();

    while (true)
    {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        feedback_ = this->get_feedback_fun();

        // publish the feedback
        as_.publishFeedback(feedback_);

        if (feedback_.finished) break;

        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    result_ = get_result_fun();

    if(result_.success)
    {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}


} // namespace as64_

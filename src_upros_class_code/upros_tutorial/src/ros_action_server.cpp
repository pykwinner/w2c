#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <upros_message/MyActionAction.h>

class MyActionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<upros_message::MyActionAction> as_;
    std::string action_name_;
    upros_message::MyActionFeedback feedback_;
    upros_message::MyActionResult result_;

public:
    MyActionServer(std::string name) : as_(nh_, name, boost::bind(&MyActionServer::executeCB, this, _1), false),
                                       action_name_(name)
    {
        as_.start();
    }

    ~MyActionServer(void) {}

    void executeCB(const upros_message::MyActionGoalConstPtr &goal)
    {
        ros::Rate r(1);
        bool success = true;

        // 执行动作
        for (int i = 1; i <= 10; i++)
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

            feedback_.progress = i * 10;
            ROS_INFO("%s: Executing, progress = %f", action_name_.c_str(), feedback_.progress);
            as_.publishFeedback(feedback_);

            r.sleep();
        }

        // 发送结果
        if (success)
        {
            result_.success = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_action_server");
    MyActionServer server("my_action");
    ros::spin();
    return 0;
}

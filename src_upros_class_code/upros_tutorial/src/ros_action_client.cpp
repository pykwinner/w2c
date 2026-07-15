#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <upros_message/MyActionAction.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "cre_client");
    ros::NodeHandle nh;

    // 创建动作客户端
    actionlib::SimpleActionClient<upros_message::MyActionAction> client("my_action", true);

    // 等待服务器启动
    ROS_INFO("Waiting for action server to start...");
    client.waitForServer();

    // 创建动作目标
    upros_message::MyActionGoal goal;
    goal.object_name = "world";

    // 发送目标并等待结果
    ROS_INFO("Sending goal...");
    client.sendGoal(goal);

    bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}


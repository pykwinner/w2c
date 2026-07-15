#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <cmath>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    move_base_msgs::MoveBaseGoal goal1;
    move_base_msgs::MoveBaseGoal goal2;
    move_base_msgs::MoveBaseGoal goal3;

    // 待发送的 desk1 目标点 在 map 坐标系下的坐标位置
    goal1.target_pose.pose.position.x = 1.90;
    goal1.target_pose.pose.position.y = -1.79;
    goal1.target_pose.pose.orientation.z = 0.0;
    goal1.target_pose.pose.orientation.w = 1.0;
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal1);
    ROS_INFO("Send Goal  1 !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 1 Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    // 待发送的 desk2 目标点 在 map 坐标系下的坐标位置
    goal2.target_pose.pose.position.x = 1.90;
    goal2.target_pose.pose.position.y = -2.99;
    goal2.target_pose.pose.orientation.z = 0.0;
    goal2.target_pose.pose.orientation.w = 1.0;
    goal2.target_pose.header.frame_id = "map";
    goal2.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal2);
    ROS_INFO("Send Goal 2 !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 2 Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    // 待发送的 home 目标点 在 map 坐标系下的坐标位置
    goal3.target_pose.pose.position.x = 0.0;
    goal3.target_pose.pose.position.y = 0.0;
    goal3.target_pose.pose.orientation.z = 0.0;
    goal3.target_pose.pose.orientation.w = 1.0;
    goal3.target_pose.header.frame_id = "map";
    goal3.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal3);
    ROS_INFO("Send Goal Home !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back !!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    return 0;
}

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

    double grab_desk_x = 1.90; // 抓取的桌子的x坐标
    double grab_desk_y = -1.77; // 抓取的桌子的y坐标

    double tag_1_put_x = 1.0;
    double tag_1_put_y = -0.77;

    double tag_2_put_x = 1.6;
    double tag_2_put_y = -0.77;

    move_base_msgs::MoveBaseGoal goal1;
    move_base_msgs::MoveBaseGoal goal2;
    move_base_msgs::MoveBaseGoal goal3;

    // 待发送的 抓取 TAG1 的目标点 在 map 坐标系下的坐标位置
    goal1.target_pose.pose.position.x = grab_desk_x;
    goal1.target_pose.pose.position.y = grab_desk_y + 0.1;
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

    // 待发送的 放置 TAG1 的目标点 在 map 坐标系下的坐标位置
    goal2.target_pose.pose.position.x = tag_1_put_x;
    goal2.target_pose.pose.position.y = tag_1_put_y;
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

    // 待发送的 抓取 TAG2 的目标点 在 map 坐标系下的坐标位置
    goal1.target_pose.pose.position.x = grab_desk_x;
    goal1.target_pose.pose.position.y = grab_desk_y - 0.1;
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

    // 待发送的 放置 TAG1 的目标点 在 map 坐标系下的坐标位置
    goal2.target_pose.pose.position.x = tag_2_put_x;
    goal2.target_pose.pose.position.y = tag_2_put_y;
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

    // 返回出发点
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

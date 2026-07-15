#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <cmath>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    tf2::Quaternion quaternion;        // 定义一个坐标变换的变量
    move_base_msgs::MoveBaseGoal goal; // 定义发布的航点变量

    // 发送漏油检测导航点,漏油点前方一小段距离
    double leak_detect_x = 0.90;  // 漏油点检测X坐标
    double leak_detect_y = -1.80; // 漏油点检测Y坐标
    quaternion.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = leak_detect_x;
    goal.target_pose.pose.position.y = leak_detect_y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    sleep(5.0);

    // 发送仪表盘检测导航点
    double panel_detect_x = 0.80;  // 表盘检测X坐标
    double panel_detect_y = -2.33; // 表盘检测Y坐标
    quaternion.setRPY(0, 0, -1.5707);
    goal.target_pose.pose.position.x = panel_detect_x;
    goal.target_pose.pose.position.y = panel_detect_y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    sleep(5.0);

    // 发送返回导航点
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
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

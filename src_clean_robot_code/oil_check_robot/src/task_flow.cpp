#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include "std_srvs/Empty.h"

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

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::ServiceClient leak_client = nh.serviceClient<std_srvs::Empty>("/leak_detect");
    ros::ServiceClient panel_client = nh.serviceClient<std_srvs::Empty>("/panel_detect");

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    tf2::Quaternion quaternion;
    move_base_msgs::MoveBaseGoal goal;
    std_srvs::Empty empty_srv;

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
        // 前进一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 10)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 放下机械臂，看看漏油
        system("rosrun oil_check_robot arm_down_node");
        // 发送检测漏油点指令
        leak_client.call(empty_srv);
        // 稍等2秒
        sleep(2.0);
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    sleep(5.0);

    // 发送仪表盘检测导航点
    double panel_detect_x = 1.0;   // 表盘检测X坐标
    double panel_detect_y = -0.77; // 表盘检测Y坐标
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
        // 先前进一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 10)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.y = 0.0;
        pub.publish(vel_msg);
        // 升起机械臂，看看表盘
        system("rosrun oil_check_robot arm_up_node");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
        // 发送检测漏油点指令
        panel_client.call(empty_srv);
        // 稍等2秒
        sleep(2.0);
    }

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

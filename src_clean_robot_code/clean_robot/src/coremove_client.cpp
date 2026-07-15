#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <upros_message/CoreMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef actionlib::SimpleActionClient<upros_message::CoreMoveAction> CoreMoveClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    CoreMoveClient cc("core_move", true);
    cc.waitForServer();

    // 弓字型路径的起始点
    double start_x = 1.1;
    double start_y = -1.0;

    // 路径生成
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_1", 10);

    // 路径覆盖矩形长宽都是 0.6 米
    double step = 0.1;
    double width = 0.6;
    double height = 0.6;

    // 填充路径
    nav_msgs::Path path;
    path.header.frame_id = "map";

    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x + start_x;
                pose.pose.position.y = y + start_y;
                path.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x + start_x;
                pose.pose.position.y = y + start_y;
                path.poses.push_back(pose);
            }
        }
    }

    path_pub.publish(path);

    // 视觉识别的点
    move_base_msgs::MoveBaseGoal goal1;
    // 弓字型路径的起始点
    move_base_msgs::MoveBaseGoal goal2;
    // 返回的出发点
    move_base_msgs::MoveBaseGoal goal3;

    // 先去 TAG 识别的点
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, -1.5707);
    goal1.target_pose.pose.position.x = 0.5;
    goal1.target_pose.pose.position.y = 0.0;
    goal1.target_pose.pose.orientation.z = quaternion.z();
    goal1.target_pose.pose.orientation.w = quaternion.w();
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

    // 去工字形路径的起始点（矩形右下角）
    goal2.target_pose.pose.position.x = start_x;
    goal2.target_pose.pose.position.y = start_y;
    goal2.target_pose.pose.orientation.z = 0.0;
    goal2.target_pose.pose.orientation.w = 1.0;
    goal2.target_pose.header.frame_id = "map";
    goal2.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal2);
    ROS_INFO("Send Goal Movebase !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    // 发布弓字型遍历点
    for (int i = 0; i < path.poses.size(); i++)
    {
        path_pub.publish(path);
        upros_message::CoreMoveGoal goal;
        goal.cmd = 1;
        goal.target_pose.pose.position.x = path.poses.at(i).pose.position.x;
        goal.target_pose.pose.position.y = path.poses.at(i).pose.position.y;
        cc.sendGoal(goal);
        ROS_INFO("Sending Goal %d of %d points", i, path.poses.size());
        cc.waitForResult();

        if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The Goal 1 Reached Successfully!!!");
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
        ROS_INFO("Goal reached %d of %d points", i, path.poses.size());
    }

    // 发送返回点
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

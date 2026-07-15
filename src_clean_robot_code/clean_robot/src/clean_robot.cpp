#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <upros_message/CoreMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

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

int tag = 0;

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    for (const auto &detection : msg->detections)
    {
        if (detection.id.size() == 1 && detection.id[0] == 1)
        {
            // detect id 1，去点1
            tag = 1;
        }
        else if (detection.id.size() == 1 && detection.id[0] == 3)
        {
            // detect id 3，去点3
            tag = 3;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    double step = 0.1; // 步长
    double width = 0.5; // 矩形的宽度
    double height = 0.5; // 矩形的长度

    double view_x = 0.0; // 视觉识别的点的x坐标   
    double view_y = 0.0; // 视觉识别的点的y坐标

    double start_x_1 = 1.79; // 路径1的起始点x坐标（矩形的右下角）
    double start_y_1 = -0.02; // 路径1的起始点y坐标（矩形的右下角）

    double start_x_2 = 1.72; // 路径2的起始点x坐标（矩形的右下角）
    double start_y_2 = 1.81; // 路径2的起始点x坐标（矩形的右下角）

    double start_x_3 = 0.06; // 路径3的起始点x坐标（矩形的右下角）
    double start_y_3 = 1.67; // 路径3的起始点x坐标（矩形的右下角）


    // 使用参数服务器获取参数，若直接在代码中获取参数，可将以下代码注释掉
    /*------------------------------------------------------------------------*/
    // nh.getParam("/clean_robot/step", step);
    // nh.getParam("/clean_robot/width", width);
    // nh.getParam("/clean_robot/height", height);

    // nh.getParam("/clean_robot/view_x", view_x);
    // nh.getParam("/clean_robot/view_y", view_y);

    // nh.getParam("/clean_robot/start_x_1", start_x_1);
    // nh.getParam("/clean_robot/start_y_1", start_y_1);
    // nh.getParam("/clean_robot/start_x_2", start_x_2);
    // nh.getParam("/clean_robot/start_y_2", start_y_2);
    // nh.getParam("/clean_robot/start_x_3", start_x_3);
    // nh.getParam("/clean_robot/start_y_3", start_y_3);
    /*-------------------------------------------------------------------------*/

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    CoreMoveClient cc("core_move", true);
    cc.waitForServer();

    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 10, tagDetectionsCallback);

    // 弓字型路径的起始点
    double start_x = 1.5;
    double start_y = -1.0;

    // 路径生成
    ros::Publisher path_1_pub = nh.advertise<nav_msgs::Path>("path_1", 10);
    ros::Publisher path_2_pub = nh.advertise<nav_msgs::Path>("path_2", 10);
    ros::Publisher path_3_pub = nh.advertise<nav_msgs::Path>("path_3", 10);

    // 第一个矩形的路径
    nav_msgs::Path path_1;
    path_1.header.frame_id = "map";
    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_1.header;
                pose.pose.position.x = x + start_x_1;
                pose.pose.position.y = y + start_y_1;
                path_1.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_1.header;
                pose.pose.position.x = x + start_x_1;
                pose.pose.position.y = y + start_y_1;
                path_1.poses.push_back(pose);
            }
        }
    }

    // 第二个矩形的路径
    nav_msgs::Path path_2;
    path_2.header.frame_id = "map";
    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_2.header;
                pose.pose.position.x = x + start_x_2;
                pose.pose.position.y = y + start_y_2;
                path_2.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_2.header;
                pose.pose.position.x = x + start_x_2;
                pose.pose.position.y = y + start_y_2;
                path_2.poses.push_back(pose);
            }
        }
    }

    // 第三个矩形的路径
    nav_msgs::Path path_3;
    path_3.header.frame_id = "map";
    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_3.header;
                pose.pose.position.x = x + start_x_3;
                pose.pose.position.y = y + start_y_3;
                path_3.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_3.header;
                pose.pose.position.x = x + start_x_3;
                pose.pose.position.y = y + start_y_3;
                path_3.poses.push_back(pose);
            }
        }
    }

    path_1_pub.publish(path_1);
    path_2_pub.publish(path_2);
    path_3_pub.publish(path_3);

    // 视觉识别的点
    move_base_msgs::MoveBaseGoal goal1;
    // 弓字型路径的起始点
    move_base_msgs::MoveBaseGoal goal2;
    // 返回的出发点
    move_base_msgs::MoveBaseGoal goal3;

    // 先去 TAG 识别的点
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, -1.5707);
    goal1.target_pose.pose.position.x = view_x;
    goal1.target_pose.pose.position.y = view_y;
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

    // 等待TAG识别
    ros::Rate rate(10);
    while (ros::ok() && tag == 0)
    {
        path_1_pub.publish(path_1);
        path_2_pub.publish(path_2);
        path_3_pub.publish(path_3);
        ros::spinOnce();
        rate.sleep();
    }

    // 订阅到了 AprilTag
    ROS_INFO("Tag Found!!!!");

    if (tag == 1)
    {
        // 去点 1 弓字形路径的起始点（矩形右下角）
        goal2.target_pose.pose.position.x = start_x_1;
        goal2.target_pose.pose.position.y = start_y_1;
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
        // 发布第 1 个区域弓字型遍历点
        for (int i = 0; i < path_1.poses.size(); i++)
        {
            path_1_pub.publish(path_1);
            upros_message::CoreMoveGoal goal;
            goal.cmd = 1;
            goal.target_pose.pose.position.x = path_1.poses.at(i).pose.position.x;
            goal.target_pose.pose.position.y = path_1.poses.at(i).pose.position.y;
            cc.sendGoal(goal);
            ROS_INFO("Sending Goal %d of %d points", i, path_1.poses.size());
            cc.waitForResult();
            if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The Goal 1 Reached Successfully!!!");
            }
            else
            {
                ROS_WARN("The Goal Planning Failed for some reason");
            }
            ROS_INFO("Goal reached %d of %d points", i, path_1.poses.size());
        }

        // 去点 2 弓字形路径的起始点（矩形右下角）
        goal2.target_pose.pose.position.x = start_x_2;
        goal2.target_pose.pose.position.y = start_y_2;
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
        // 发布第 2 个区域弓字型遍历点
        for (int i = 0; i < path_2.poses.size(); i++)
        {
            path_2_pub.publish(path_2);
            upros_message::CoreMoveGoal goal;
            goal.cmd = 1;
            goal.target_pose.pose.position.x = path_2.poses.at(i).pose.position.x;
            goal.target_pose.pose.position.y = path_2.poses.at(i).pose.position.y;
            cc.sendGoal(goal);
            ROS_INFO("Sending Goal %d of %d points", i, path_2.poses.size());
            cc.waitForResult();
            if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The Goal 1 Reached Successfully!!!");
            }
            else
            {
                ROS_WARN("The Goal Planning Failed for some reason");
            }
            ROS_INFO("Goal reached %d of %d points", i, path_2.poses.size());
        }

        // 去点 3 弓字形路径的起始点（矩形右下角）
        goal2.target_pose.pose.position.x = start_x_3;
        goal2.target_pose.pose.position.y = start_y_3;
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
        // 发布第 3 个区域弓字型遍历点
        for (int i = 0; i < path_3.poses.size(); i++)
        {
            path_3_pub.publish(path_3);
            upros_message::CoreMoveGoal goal;
            goal.cmd = 1;
            goal.target_pose.pose.position.x = path_3.poses.at(i).pose.position.x;
            goal.target_pose.pose.position.y = path_3.poses.at(i).pose.position.y;
            cc.sendGoal(goal);
            ROS_INFO("Sending Goal %d of %d points", i, path_3.poses.size());
            cc.waitForResult();
            if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The Goal 1 Reached Successfully!!!");
            }
            else
            {
                ROS_WARN("The Goal Planning Failed for some reason");
            }
            ROS_INFO("Goal reached %d of %d points", i, path_3.poses.size());
        }
    }

    if (tag == 3)
    {
        // 去点 3 弓字形路径的起始点（矩形右下角）
        goal2.target_pose.pose.position.x = start_x_3;
        goal2.target_pose.pose.position.y = start_y_3;
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
        // 发布第 3 个区域弓字型遍历点
        for (int i = 0; i < path_3.poses.size(); i++)
        {
            path_3_pub.publish(path_3);
            upros_message::CoreMoveGoal goal;
            goal.cmd = 1;
            goal.target_pose.pose.position.x = path_3.poses.at(i).pose.position.x;
            goal.target_pose.pose.position.y = path_3.poses.at(i).pose.position.y;
            cc.sendGoal(goal);
            ROS_INFO("Sending Goal %d of %d points", i, path_3.poses.size());
            cc.waitForResult();
            if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The Goal 1 Reached Successfully!!!");
            }
            else
            {
                ROS_WARN("The Goal Planning Failed for some reason");
            }
            ROS_INFO("Goal reached %d of %d points", i, path_3.poses.size());
        }

        // 去点 2 弓字形路径的起始点（矩形右下角）
        goal2.target_pose.pose.position.x = start_x_2;
        goal2.target_pose.pose.position.y = start_y_2;
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
        // 发布第 2 个区域弓字型遍历点
        for (int i = 0; i < path_2.poses.size(); i++)
        {
            path_2_pub.publish(path_2);
            upros_message::CoreMoveGoal goal;
            goal.cmd = 1;
            goal.target_pose.pose.position.x = path_2.poses.at(i).pose.position.x;
            goal.target_pose.pose.position.y = path_2.poses.at(i).pose.position.y;
            cc.sendGoal(goal);
            ROS_INFO("Sending Goal %d of %d points", i, path_2.poses.size());
            cc.waitForResult();
            if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The Goal 1 Reached Successfully!!!");
            }
            else
            {
                ROS_WARN("The Goal Planning Failed for some reason");
            }
            ROS_INFO("Goal reached %d of %d points", i, path_2.poses.size());
        }

        // 去点 1 弓字形路径的起始点（矩形右下角）
        goal2.target_pose.pose.position.x = start_x_1;
        goal2.target_pose.pose.position.y = start_y_1;
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
        // 发布第 1 个区域弓字型遍历点
        for (int i = 0; i < path_1.poses.size(); i++)
        {
            path_1_pub.publish(path_1);
            upros_message::CoreMoveGoal goal;
            goal.cmd = 1;
            goal.target_pose.pose.position.x = path_1.poses.at(i).pose.position.x;
            goal.target_pose.pose.position.y = path_1.poses.at(i).pose.position.y;
            cc.sendGoal(goal);
            ROS_INFO("Sending Goal %d of %d points", i, path_1.poses.size());
            cc.waitForResult();
            if (cc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The Goal 1 Reached Successfully!!!");
            }
            else
            {
                ROS_WARN("The Goal Planning Failed for some reason");
            }
            ROS_INFO("Goal reached %d of %d points", i, path_1.poses.size());
        }
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

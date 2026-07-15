#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include "std_srvs/Empty.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
        else if (detection.id.size() == 1 && detection.id[0] == 1)
        {
            // detect id 3，去点3
            tag = 2;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 10, tagDetectionsCallback);

    ros::ServiceClient up_client = nh.serviceClient<std_srvs::Empty>("/up_service");
    ros::ServiceClient down_client = nh.serviceClient<std_srvs::Empty>("/down_service");

    // 视觉识别的点
    move_base_msgs::MoveBaseGoal goal1;
    // 货架搬运点
    move_base_msgs::MoveBaseGoal goal2;
    // 货架配送点
    move_base_msgs::MoveBaseGoal goal3;
    // 返回的出发点
    move_base_msgs::MoveBaseGoal goal4;

    // 先去 TAG 识别的点
    double view_x = 0.5; // 视觉识别的点的x坐标
    double view_y = 0.0; // 视觉识别的点的y坐标
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
        ros::spinOnce();
        rate.sleep();
    }

    if (tag == 1)
    {
        // 订阅到了 AprilTag
        ROS_INFO("Tag Found 1!!!!");
        // 去1号货架的摆渡点
        double start_x_1 = 1.79;
        double start_y_1 = 0.5;
        quaternion.setRPY(0, 0, 0);
        goal2.target_pose.pose.position.x = start_x_1;
        goal2.target_pose.pose.position.y = start_y_1;
        goal2.target_pose.pose.orientation.z = quaternion.z();
        goal2.target_pose.pose.orientation.w = quaternion.w();
        goal2.target_pose.header.frame_id = "map";
        goal2.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal2);
        ROS_INFO("Send Goal Movebase !!!");
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The MoveBase Goal Reached Successfully!!!");
            // 先前进一小段,大约30cm
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.1;
            int count = 0;
            ros::Rate loop_rate(10);
            while (ros::ok() && count < 30)
            {
                pub.publish(vel_msg);
                ros::spinOnce();
                loop_rate.sleep();
                count++;
            }
            // 停下
            vel_msg.linear.x = 0.0;
            pub.publish(vel_msg);
            // 抬起顶升
            std_srvs::Empty empty_srv;
            up_client.call(empty_srv);
            sleep(5.0);
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
        // 先后退一小段,大约30cm，防止规划时发生碰撞
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = -0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 30)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);

        // 去放置点的摆渡点
        double put_x = 0.1;
        double put_y = 1.8;
        quaternion.setRPY(0, 0, 3.1415926);
        goal3.target_pose.pose.position.x = put_x;
        goal3.target_pose.pose.position.y = put_y;
        goal3.target_pose.pose.orientation.z = quaternion.z();
        goal3.target_pose.pose.orientation.w = quaternion.w();
        goal3.target_pose.header.frame_id = "map";
        goal3.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal3);
        ROS_INFO("Send Goal Movebase !!!");
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
            vel_msg.linear.x = 0.0;
            pub.publish(vel_msg);
            // 放下顶升
            std_srvs::Empty empty_srv;
            down_client.call(empty_srv);
            sleep(5.0);
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
        // 先后退一小段,大约30cm，防止规划时发生碰撞
        vel_msg.linear.x = -0.1;
        count = 0;
        while (ros::ok() && count < 30)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
    }

    if (tag == 2)
    {
        // 订阅到了 AprilTag
        ROS_INFO("Tag Found 2!!!!");
        // 去2号货架的摆渡点
        double start_x_2 = 1.79;
        double start_y_2 = 1.0;
        quaternion.setRPY(0, 0, 1.5707);
        goal2.target_pose.pose.position.x = start_x_2;
        goal2.target_pose.pose.position.y = start_y_2;
        goal2.target_pose.pose.orientation.z = quaternion.z();
        goal2.target_pose.pose.orientation.w = quaternion.w();
        goal2.target_pose.header.frame_id = "map";
        goal2.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal2);
        ROS_INFO("Send Goal Movebase !!!");
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
            vel_msg.linear.x = 0.0;
            pub.publish(vel_msg);
            // 抬起顶升
            std_srvs::Empty empty_srv;
            up_client.call(empty_srv);
            sleep(5.0);
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
        // 先后退一小段,大约30cm，防止规划时发生碰撞
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = -0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 30)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);

        // 去放置点的摆渡点
        double put_x = 0.06;
        double put_y = 1.67;
        quaternion.setRPY(0, 0, 3.1415926);
        goal3.target_pose.pose.position.x = put_x;
        goal3.target_pose.pose.position.y = put_y;
        goal3.target_pose.pose.orientation.z = quaternion.z();
        goal3.target_pose.pose.orientation.w = quaternion.w();
        goal3.target_pose.header.frame_id = "map";
        goal3.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal3);
        ROS_INFO("Send Goal Movebase !!!");
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
            vel_msg.linear.x = 0.0;
            pub.publish(vel_msg);
            // 放下顶升
            std_srvs::Empty empty_srv;
            down_client.call(empty_srv);
            sleep(5.0);
        }
        else
        {
            ROS_WARN("The Goal Planning Failed for some reason");
        }
        // 先后退一小段,大约30cm，防止规划时发生碰撞
        vel_msg.linear.x = -0.1;
        count = 0;
        while (ros::ok() && count < 30)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
    }

    // 发送返回点
    goal4.target_pose.pose.position.x = 0.0;
    goal4.target_pose.pose.position.y = 0.0;
    goal4.target_pose.pose.orientation.z = 0.0;
    goal4.target_pose.pose.orientation.w = 1.0;
    goal4.target_pose.header.frame_id = "map";
    goal4.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal4);
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

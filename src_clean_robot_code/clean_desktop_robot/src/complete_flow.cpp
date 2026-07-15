#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

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

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    double grab_1_x = 1.90;  // movebase目标点x，第一张桌子前方的导航点
    double grab_1_y = -1.83; // movebase目标点y，第一张桌子前方的导航点
    
    double grab_2_x = 1.90;  // movebase目标点x，第二张桌子前方的导航点
    double grab_2_y = -3.05; // movebase目标点y，第二张桌子前方的导航点

    double offset_left = 0.4;  // 左侧偏移量，单位为米
    double offset_right = 0.4; // 右侧偏移量，单位为米

    // 使用参数服务器获取参数，若不需要参数服务器，可将以下代码注释掉
    /*------------------------------------------------------------------------*/
    // nh.getParam("/complete_flow_node/grab_1_x", grab_1_x);
    // nh.getParam("/complete_flow_node/grab_1_y", grab_1_y);
    // nh.getParam("/complete_flow_node/grab_2_x", grab_2_x);
    // nh.getParam("/complete_flow_node/grab_2_y", grab_2_y);
    // nh.getParam("/complete_flow_node/offset_left", offset_left);
    // nh.getParam("/complete_flow_node/offset_right", offset_right);
    /*------------------------------------------------------------------------*/


    move_base_msgs::MoveBaseGoal goal;

    // 发送抓取导航点,桌子前方一小段距离
    goal.target_pose.pose.position.x = grab_1_x;
    goal.target_pose.pose.position.y = grab_1_y;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
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
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取物块
        system("roslaunch clean_desktop_robot arm_grab.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
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

    // 发送放置导航点
    goal.target_pose.pose.position.x = grab_1_x;
    goal.target_pose.pose.position.y = grab_1_y - offset_right; // 放置点位于抓取点右侧约0.4米
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 放置物块
        system("roslaunch clean_desktop_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // 先后退一小段,大约30cm
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

    // 发送抓取导航点,桌子前方一小段距离
    goal.target_pose.pose.position.x = grab_2_x;
    goal.target_pose.pose.position.y = grab_2_y;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
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
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取物块
        system("roslaunch clean_desktop_robot arm_grab.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
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

    // 发送放置导航点
    goal.target_pose.pose.position.x = grab_2_x;
    goal.target_pose.pose.position.y = grab_2_y + offset_left; // 放置点位于抓取点左侧约0.4米
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 放置物块
        system("roslaunch clean_desktop_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // 先后退一小段,大约30cm
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

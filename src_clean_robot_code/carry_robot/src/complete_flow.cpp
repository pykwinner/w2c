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

    double grab_desk_x = 1.90;  // 抓取的桌子的x坐标
    double grab_desk_y = -1.80; // 抓取的桌子的y坐标

    double tag_1_put_x = 1.0; // 放置tag1的x坐标
    double tag_1_put_y = -0.77; // 放置tag1的y坐标

    double tag_2_put_x = 1.6; // 放置tag2的x坐标
    double tag_2_put_y = -0.77; // 放置tag2的y坐标

    double offset_left = 0.1; // tag1在桌子中轴线左侧10cm
    double offset_right = 0.1; // tag2在桌子中轴线右侧10cm

    // 使用参数服务器获取参数，若不需要参数服务器，可将以下代码注释掉
    /*------------------------------------------------------------------------*/
    // nh.getParam("/w4a_complete_flow_node/grab_desk_x", grab_desk_x);
    // nh.getParam("/w4a_complete_flow_node/grab_desk_y", grab_desk_y);
    // nh.getParam("/w4a_complete_flow_node/tag_1_put_x", tag_1_put_x);
    // nh.getParam("/w4a_complete_flow_node/tag_1_put_y", tag_1_put_y);
    // nh.getParam("/w4a_complete_flow_node/tag_2_put_x", tag_2_put_x);
    // nh.getParam("/w4a_complete_flow_node/tag_2_put_y", tag_2_put_y);
    // nh.getParam("/w4a_complete_flow_node/offset_left", offset_left);
    // nh.getParam("/w4a_complete_flow_node/offset_right", offset_right);
    /*------------------------------------------------------------------------*/

    // tf2::Quaternion quaternion;
    // quaternion.setRPY(0, 0, 1.5707);

    move_base_msgs::MoveBaseGoal goal;

    // 发送抓取导航点,桌子前方一小段距离
    goal.target_pose.pose.position.x = grab_desk_x;
    goal.target_pose.pose.position.y = grab_desk_y + offset_left; // TAG1物块位置在桌子中轴线左侧10cm
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
        // 抓取TAG位1的物块
        system("roslaunch carry_robot arm_grab_1.launch");
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
    goal.target_pose.pose.position.x = tag_1_put_x;
    goal.target_pose.pose.position.y = tag_1_put_y;
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
        // 先左移一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.y = 0.1;
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
        // 放置物块
        system("roslaunch carry_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // 先右移一小段,大约30cm，防止再规划碰撞
    vel_msg.linear.y = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);

    // 发送抓取导航点,桌子前方一小段距离
    goal.target_pose.pose.position.x = grab_desk_x;
    goal.target_pose.pose.position.y = grab_desk_y - offset_right; // TAG2物块位置在桌子中轴线右侧10cm
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
        // 抓取TAG位1的物块
        system("roslaunch carry_robot arm_grab_2.launch");
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
    goal.target_pose.pose.position.x = tag_2_put_x;
    goal.target_pose.pose.position.y = tag_2_put_y;
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
        // 先左移一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.y = 0.1;
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
        // 放置物块
        system("roslaunch carry_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // 先右移一小段,大约30cm，防止再规划碰撞
    vel_msg.linear.y = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.y = 0.0;
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

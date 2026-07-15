#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

bool got_finish = false;
double odom_distance = 0;

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    double odom_x = odom_msg->pose.pose.position.x;
    odom_distance = odom_x;
    if (odom_x > 1.0)
    {
        if (!got_finish)
        {
            got_finish = true;
            ROS_INFO("Finish!!!!!!!!");
        }
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_listener");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 订阅Odom数据
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, odom_callback);
    // 运动控制
    ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist cmd_vel;
    // 循环等待回调函数
    while (ros::ok())
    {
        if (got_finish)
        {
            cmd_vel.linear.x = 0.0;
        }
        else
        {
            cmd_vel.linear.x = 0.25;
        }
        cmd_pub_.publish(cmd_vel);
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    return 0;
}

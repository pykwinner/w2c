#include <ros/ros.h>
#include <tf/transform_listener.h> // tf的头文件
#include <nav_msgs/Odometry.h> // 包含Odometry消息头文件

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_listener"); // 初始化ROS节点
    ros::NodeHandle node; // 创建ROS节点句柄

    // 创建Transform监听器
    tf::TransformListener listener;

    // 创建一个发布者，发布/odom主题，消息类型为nav_msgs::Odometry
    ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("/odom_app", 10);

    ros::Rate rate(10.0); // 设置循环频率为10Hz

    while (node.ok())
    { // 循环直到ROS节点停止
        // 创建StampedTransform对象，用于存储变换信息
        tf::StampedTransform transform;
        try
        {
            // 获取base_link在map坐标系下的变换
            listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ros::Duration(1.0).sleep(); // 如果获取变换失败，等待1秒再继续
            continue;
        }

        // 创建Odometry消息
        nav_msgs::Odometry odom_msg;
        // 设置消息的header
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        // 设置位置信息
        odom_msg.pose.pose.position.x = transform.getOrigin().x();
        odom_msg.pose.pose.position.y = transform.getOrigin().y();
        odom_msg.pose.pose.position.z = transform.getOrigin().z();

        // 设置姿态信息
        tf::Quaternion q = transform.getRotation();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 由于没有速度信息，这里简单设置为0
        odom_msg.twist.twist.linear.x = 0;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.x = 0;
        odom_msg.twist.twist.angular.y = 0;
        odom_msg.twist.twist.angular.z = 0;

        // 发布Odometry消息
        odom_pub.publish(odom_msg);

        // 使用getEulerYPR获取所有欧拉角
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getEulerYPR(yaw, pitch, roll);

        // 打印base_link在map坐标系下的位姿信息
        ROS_INFO("base_link in map coordinate system: x = %f, y = %f, z = %f, roll = %f, pitch = %f, yaw = %f",
                 transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), roll, pitch, yaw);

        rate.sleep(); // 等待一段时间，以满足循环频率
    }
    return 0; // 程序结束
}    

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16MultiArray.h"

ros::Publisher cmd_vel_pub;

int last_bump_forward = 0;

void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
    // 上一帧没有触发碰撞，这一帧触发了，后退一段距离
    if (msg->data[0] == 1 && last_bump_forward == 0)
    {
        ROS_INFO("前方发生碰撞！");
        geometry_msgs::Twist cmd_vel;
        for (int i = 0; i < 20; i++)
        {
            cmd_vel.linear.x = -0.2; // 设置后退速度
            cmd_vel_pub.publish(cmd_vel);
            ros::Duration(0.05).sleep();
        }
        cmd_vel.linear.x = 0.0; // 停
        cmd_vel_pub.publish(cmd_vel);
    }

    // 给上一帧碰撞赋值
    last_bump_forward = msg->data[0];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bump_sensor_subscriber");
    ros::NodeHandle n;

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("/robot/bump_sensor", 1000, bumpCallback);

    ros::spin();

    return 0;
}
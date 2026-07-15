#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmdVelPub;
float firstRange, lastRange;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int num_points = (scan->angle_max - scan->angle_min) / scan->angle_increment;
    firstRange = scan->ranges[0];
    lastRange = scan->ranges[num_points - 1];
    float meanRange = (firstRange + lastRange) / 2.0;
    ROS_INFO_STREAM("Mean range: " << meanRange);

    geometry_msgs::Twist cmdVelMsg;
    if (meanRange >= 8.0)
    {
        cmdVelMsg.linear.x = 0.0;
    }
    else
    {
        cmdVelMsg.linear.x = 0.3;
    }
    cmdVelPub.publish(cmdVelMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_scan_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);
    cmdVelPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::spin();
    return 0;
}
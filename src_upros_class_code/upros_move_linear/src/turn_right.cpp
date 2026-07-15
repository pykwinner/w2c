#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(10); // 10Hz，即每 100 毫秒一次

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = -0.5;

    int count = 0;
    while (ros::ok() && count < 50)
    { // 100 * 50ms = 5000ms = 5s
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);

    return 0;
}
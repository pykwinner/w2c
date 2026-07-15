#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <angles/angles.h>

double current_yaw;

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // 获取IMU数据
    geometry_msgs::Vector3 linear_acceleration = imu_msg->linear_acceleration;
    geometry_msgs::Vector3 angular_velocity = imu_msg->angular_velocity;
    geometry_msgs::Quaternion orientation = imu_msg->orientation;
    // 打印IMU数据
    ROS_INFO("Linear acceleration: x=%f, y=%f, z=%f", linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
    ROS_INFO("Angular velocity: x=%f, y=%f, z=%f", angular_velocity.x, angular_velocity.y, angular_velocity.z);
    ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f", orientation.x, orientation.y, orientation.z, orientation.w);
    double current_angle = tf2::getYaw(orientation);
    current_yaw = current_angle;
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "imu_listener");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 订阅IMU数据
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, imu_callback);
    //发布速度控制的Publisher
    ros::Publisher rotate_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //设定起始角度
    double start_yaw = current_yaw;
    //判断是否旋转180度的标志位
    bool got_180 = false;

    // 循环等待回调函数
    while (ros::ok() && !got_180)
    {
        if (!got_180)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = 0.5;
            cmd_vel.linear.x = 0.0;
            //以0.5rad/s的速度左转
            rotate_pub_.publish(cmd_vel);
            //当前角度对于目标角度的差距
            double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_yaw, start_yaw + M_PI));
            //当差距小于0.1rad时，停止自旋
            if (distance_to_180 < 0.1)
            {
              got_180 = true;
              cmd_vel.angular.z = 0.0;
              rotate_pub_.publish(cmd_vel);
            }
        }
        ros::spinOnce();
        ros::Duration(0.05).sleep();    
    }
    
    return 0;
}

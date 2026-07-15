#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // 获取IMU-加速度计数据
    geometry_msgs::Vector3 linear_acceleration = imu_msg->linear_acceleration;
    //获取IMU-陀螺仪数据
    geometry_msgs::Vector3 angular_velocity = imu_msg->angular_velocity;
    //获取IMU-姿态数据
    geometry_msgs::Quaternion orientation = imu_msg->orientation;
    // 打印IMU数据
    ROS_INFO("Linear acceleration: x=%f, y=%f, z=%f", linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
    ROS_INFO("Angular velocity: x=%f, y=%f, z=%f", angular_velocity.x, angular_velocity.y, angular_velocity.z);
    ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f", orientation.x, orientation.y, orientation.z, orientation.w);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "imu_listener");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 订阅IMU数据
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, imu_callback);
    // 循环等待回调函数
    ros::spin();
    return 0;
}


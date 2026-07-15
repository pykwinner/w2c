#include <ros/ros.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "ros_logging_example");
    // 创建ROS节点句柄
    ros::NodeHandle nh;
    // 打印不同级别的日志消息
    ROS_DEBUG("This is a DEBUG message.");
    ROS_INFO("This is an INFO message.");
    ROS_WARN("This is a WARNING message.");
    ROS_ERROR("This is an ERROR message.");
    ROS_FATAL("This is a FATAL message.");
    // 运行ROS事件循环
    ros::spin();
    return 0;
}

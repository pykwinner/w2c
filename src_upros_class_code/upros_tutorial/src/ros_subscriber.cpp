#include <ros/ros.h>
#include "upros_message/MyMessage.h"

void callback(const upros_message::MyMessage::ConstPtr &msg) {
    std::string str = "callback   key : " + std::to_string(msg->key) + " , value : " + msg->value;
    std::cout << str << std::endl;
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "my_subscriber");

    ROS_INFO("start a subscriber node");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 定义一个订阅者对象
    ros::Subscriber sub = nh.subscribe("my_topic", 10, callback);

    // 运行节点
    ros::spin();

    return 0;
}

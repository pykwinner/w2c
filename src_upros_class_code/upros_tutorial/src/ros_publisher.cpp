#include <ros/ros.h>
#include <upros_message/MyMessage.h>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "my_publisher");

    ROS_INFO("start a publisher node ..");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 定义一个发布者对象
    ros::Publisher pub = nh.advertise<upros_message::MyMessage>("my_topic", 10);

    //定义一个ros频率，间歇1.0秒
    ros::Rate rate(1.0);

    while (ros::ok()) {
        // 创建一个消息对象并填充数据
        upros_message::MyMessage msg;
        msg.key = 1;
        msg.value = "Hello, ROS!";

        // 发布消息
        pub.publish(msg);
        
        //间歇休息1秒
        rate.sleep();
    }
    return 0;
}

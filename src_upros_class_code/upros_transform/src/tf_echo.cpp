#include <ros/ros.h>
#include <tf/transform_listener.h> // tf的头文件

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tf_listener"); // 初始化ROS节点

    ros::NodeHandle node; // 创建ROS节点句柄

    tf::TransformListener listener; // 创建Transform监听器

    ros::Rate rate(10.0); // 设置循环频率为10Hz

    while (node.ok())
    { // 循环直到ROS节点停止

        // 创建StampedTransform对象，用于存储变换信息
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("map", "base_link", ros::Time(0), transform); // 获取base_link在map坐标系下的变换
        }
        catch (tf::TransformException ex)
        {
            ros::Duration(1.0).sleep(); // 如果获取变换失败，打印错误信ROS_ERROR("%s",ex.what()),等待1秒再继续continue;
        }

        tf::Quaternion q = transform.getRotation();

        // 使用getEulerYPR获取所有欧拉角
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getEulerYPR(yaw, pitch, roll);

        // 打印base_link在odom坐标系下的位姿信息
        ROS_INFO("base_link in map coordinate system: x = %f, y = %f, z = %f, roll = %f, pitch = %f, yaw = %f",
                 transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), roll, pitch, yaw);

        rate.sleep(); // 等待一段时间，以满足循环频率
    }
    return 0; // 程序结束
}

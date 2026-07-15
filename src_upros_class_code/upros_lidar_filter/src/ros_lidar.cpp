#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
    // 获取激光点的个数
    int num_points = (scan->angle_max - scan->angle_min) / scan->angle_increment;
    ROS_INFO_STREAM("Number of points: " << num_points);
    // 获取第一个激光点的距离
    float first_range = scan->ranges[0];
    ROS_INFO_STREAM("First range: " << first_range);
    // 获取中间激光点的距离
    float middle_range = scan->ranges[num_points/2];
    ROS_INFO_STREAM("Middle range: " << middle_range);
    // 获取1/4激光点的距离
    float half_middle_range = scan->ranges[num_points/4];
    ROS_INFO_STREAM("Last range: " << half_middle_range);
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "laser_scan_listener");
    // 创建节点句柄
    ros::NodeHandle n;
    // 订阅/scan话题
    ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);
    // 循环等待回调函数
    ros::spin();
    return 0;
}

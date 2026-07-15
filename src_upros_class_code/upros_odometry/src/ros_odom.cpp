#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class OdomTracker
{
public:
    OdomTracker()
    {
        // 初始化ROS节点
        ros::NodeHandle nh;
        // 初始化订阅和发布的话题
        odom_sub_ = nh.subscribe("/odom", 10, &OdomTracker::odomCallback, this);
        path_pub_ = nh.advertise<nav_msgs::Path>("/odom_path", 10);
        // 设置默认的机器人位姿
        current_pose_.pose.pose.position.x = 0.0;
        current_pose_.pose.pose.position.y = 0.0;
        current_pose_.pose.pose.orientation.w = 1.0;
    }

private:
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher path_pub_;
    nav_msgs::Odometry current_pose_;
    nav_msgs::Path odom_path_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 更新机器人的位姿
        current_pose_ = *msg;
        // 将机器人的位姿添加到轨迹中
        odom_path_.header = current_pose_.header;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = current_pose_.pose.pose.position.x;
        pose.pose.position.y = current_pose_.pose.pose.position.y;
        odom_path_.poses.push_back(pose);
        // 发布轨迹信息
        path_pub_.publish(odom_path_);
    }
};

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_tracker");
    // 创建OdomTracker对象
    OdomTracker tracker;
    // 进入ROS循环
    ros::spin();
    return 0;
}

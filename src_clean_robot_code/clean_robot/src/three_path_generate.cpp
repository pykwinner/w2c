#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "three_path_generate");
    ros::NodeHandle nh;

    ros::Publisher path_1_pub = nh.advertise<nav_msgs::Path>("path_1", 10);
    ros::Publisher path_2_pub = nh.advertise<nav_msgs::Path>("path_2", 10);
    ros::Publisher path_3_pub = nh.advertise<nav_msgs::Path>("path_3", 10);

    double step = 0.1;
    double width = 0.5;
    double height = 0.5;

    double start_x_1 = 1.1; // 路径1的起始点x坐标（矩形的右下角）
    double start_y_1 = -0.7; // 路径1的起始点y坐标（矩形的右下角）

    double start_x_2 = 1.1; // 路径2的起始点x坐标（矩形的右下角）
    double start_y_2 = -1.7; // 路径2的起始点x坐标（矩形的右下角）

    double start_x_3 = 1.1; // 路径3的起始点x坐标（矩形的右下角）
    double start_y_3 = -2.7; // 路径3的起始点x坐标（矩形的右下角）

    // 第一个矩形的路径
    nav_msgs::Path path_1;
    path_1.header.frame_id = "map";
    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_1.header;
                pose.pose.position.x = x + start_x_1;
                pose.pose.position.y = y + start_y_1;
                path_1.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_1.header;
                pose.pose.position.x = x + start_x_1;
                pose.pose.position.y = y + start_y_1;
                path_1.poses.push_back(pose);
            }
        }
    }

    //第二个矩形的路径
    nav_msgs::Path path_2;
    path_2.header.frame_id = "map";
    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_2.header;
                pose.pose.position.x = x + start_x_2;
                pose.pose.position.y = y + start_y_2;
                path_2.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_2.header;
                pose.pose.position.x = x + start_x_2;
                pose.pose.position.y = y + start_y_2;
                path_2.poses.push_back(pose);
            }
        }
    }

    //第三个矩形的路径
    nav_msgs::Path path_3;
    path_3.header.frame_id = "map";
    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_3.header;
                pose.pose.position.x = x + start_x_3;
                pose.pose.position.y = y + start_y_3;
                path_3.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path_3.header;
                pose.pose.position.x = x + start_x_3;
                pose.pose.position.y = y + start_y_3;
                path_3.poses.push_back(pose);
            }
        }
    }

    //路径发布出去
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_1_pub.publish(path_1);
        path_2_pub.publish(path_2);
        path_3_pub.publish(path_3);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
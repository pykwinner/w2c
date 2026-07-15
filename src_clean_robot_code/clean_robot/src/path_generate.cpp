#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generate");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_generate", 10);

    double step = 0.1;
    double width = 0.5;
    double height = 0.5;

    nav_msgs::Path path;
    path.header.frame_id = "map";

    // 在 0.5m * 0.5m 的矩形范围内，生成全覆盖路径
    for (double y = 0; y < height; y += step)
    {
        if ((int(y / step)) % 2 == 0)
        {
            for (double x = 0; x < width; x += step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                path.poses.push_back(pose);
            }
        }
        else
        {
            for (double x = width - step; x >= 0; x -= step)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                path.poses.push_back(pose);
            }
        }
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //将生成的路径发布出去
        path_pub.publish(path);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
#include "ros/ros.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "c_node");

    ROS_INFO("Hello ROS C++ !!!");

    ros::spin();
}

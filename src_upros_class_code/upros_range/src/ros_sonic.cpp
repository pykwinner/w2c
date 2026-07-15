#include <ros/ros.h>
#include <sensor_msgs/Range.h>

void rangeCallback1(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Distance Front: %f", msg->range);
}

void rangeCallback2(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Distance Left: %f", msg->range);
}

void rangeCallback3(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Distance Right: %f", msg->range);
}

void rangeCallback4(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("Distance Back: %f", msg->range);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "range_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::Range>("/front_sonar", 10, rangeCallback1);
    ros::Subscriber sub_2 = nh.subscribe<sensor_msgs::Range>("/left_sonar", 10, rangeCallback2);
    ros::Subscriber sub_3 = nh.subscribe<sensor_msgs::Range>("/right_sonar", 10, rangeCallback3);
    ros::Subscriber sub_4 = nh.subscribe<sensor_msgs::Range>("/back_sonar", 10, rangeCallback4);
    ros::spin();
    return 0;
}

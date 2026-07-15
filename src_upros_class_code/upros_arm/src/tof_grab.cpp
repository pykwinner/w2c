#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "upros_arm/upros_arm_driver.h"

int current_tof_value = 100000;
int last_tof_value = 100000;

bool grab = false;

void rangeCallback4(const sensor_msgs::Range::ConstPtr &msg)
{
    // 如果之前tof没东西，突然有东西，那么执行逆运算抓取
    ROS_INFO("Distance TOF: %f", msg->range);
    current_tof_value = int(msg->range * 1000);
    if (last_tof_value >= 600 && current_tof_value <= 300)
    {
        grab = true;
    }
    last_tof_value = current_tof_value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "range_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub_4 = nh.subscribe<sensor_msgs::Range>("/us/tof1", 10, rangeCallback4);

    UPROS_ARM arm;

    ros::Rate rate(10);
    while (ros::ok())
    {
        if (grab)
        {
            int x = 0;
            int y = current_tof_value + 95;
            int z = 65;

            if (arm.inverseFind(x, y, z))
            {
                ROS_INFO("Find Soulition!!!!");
            }

            sleep(1.0);

            arm.inverseMoveToGrab();
            grab = false;
            sleep(2.0);
            arm.claw_close();
            sleep(2.0);
            arm.go_home();
            sleep(1.0);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
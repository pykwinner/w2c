#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "claw_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    std_srvs::Empty empty_srv;
    // arm_zero_client.call(empty_srv);
    sleep(2.0);

    // 夹爪闭合
    arm_grab_client.call(empty_srv);
    sleep(5.0);

    // 夹爪张开
    arm_release_client.call(empty_srv);
    sleep(5.0);

    ros::shutdown();

    return 0;
}
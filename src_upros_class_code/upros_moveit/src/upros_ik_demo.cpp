#include "upros_moveit/upros_demo.h"

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    UPROS upros("arm_group");

    sleep(3.0);

    if (upros.go_to_pose(0.0, 0.3, 0.1, -90, 0, 0))
    {
        ROS_INFO("Move Pose Succeed!!!");
    }

    ros::shutdown();
}

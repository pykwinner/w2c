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

    std::vector<double> joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    upros.go_to_joint(joints);

    ros::shutdown();
}

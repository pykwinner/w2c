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

    upros.move_Home();

    sleep(3.0);

    upros.move_Grab();

    geometry_msgs::Pose current_pose = upros.get_current_pose();

    std::cout << "CurrentPose X: " << current_pose.position.x << std::endl;
    std::cout << "CurrentPose Y: " << current_pose.position.y << std::endl;
    std::cout << "CurrentPose Z: " << current_pose.position.z << std::endl;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q1;
    q1.x = current_pose.orientation.x;
    q1.y = current_pose.orientation.y;
    q1.z = current_pose.orientation.z;
    q1.w = current_pose.orientation.w;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(q1, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    std::cout << "CurrentPose R: " << roll << std::endl;
    std::cout << "CurrentPose P: " << pitch << std::endl;
    std::cout << "CurrentPose Y: " << yaw << std::endl;

    sleep(1.0);

    ros::shutdown();
}

#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_close");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    //第一步，运动到放置点
    upros_message::ArmPosition move_srv;
    move_srv.request.x = 0;
    move_srv.request.y = 300.0;
    move_srv.request.z = -20.0;
    arm_move_close_client.call(move_srv);
    sleep(5.0);

    std_srvs::Empty empty_srv;
    
    //第二步，打开夹爪 
    arm_release_client.call(empty_srv);
    sleep(5.0);

    //第三步，返回零位
    arm_zero_client.call(empty_srv);
    sleep(5.0);

    ros::shutdown();

    return 0;
}

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

    //运动到距离机械臂底座前方300mm，高度0mm的位置（可看到漏油）
    upros_message::ArmPosition move_srv;
    move_srv.request.x = 0;
    move_srv.request.y = 300.0;
    move_srv.request.z = 0.0;
    arm_move_close_client.call(move_srv);
    sleep(5.0);
    
    ros::shutdown();

    return 0;
}
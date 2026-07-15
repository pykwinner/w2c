#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "put_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/control_center/arm_pos_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/control_center/release_service");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/control_center/zero_service");

    // 放置的目标点，单位毫米，以机械臂基座上自转轴为基点，x左正右负，y前正后负，z上正下负
    int target_put_x = 240;
    int target_put_y = 0;
    int target_put_z = 0;

    upros_message::ArmPosition move_srv;
    //第一步，运动到放置点-高
    move_srv.request.x = target_put_x;
    move_srv.request.y = target_put_y;
    move_srv.request.z = target_put_z + 120;
    arm_move_close_client.call(move_srv);
    sleep(5.0);

    //第二步，运动到放置点-低
    move_srv.request.x = target_put_x;
    move_srv.request.y = target_put_y;
    move_srv.request.z = target_put_z - 50;
    arm_move_close_client.call(move_srv);
    sleep(5.0);

    std_srvs::Empty empty_srv;
    
    //第三步，打开气泵 
    arm_release_client.call(empty_srv);
    sleep(5.0);

   arm_zero_client.call(empty_srv);

    ros::shutdown();

    return 0;
}

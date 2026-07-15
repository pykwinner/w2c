#include "upros_message/ArmPosition.h"
#include "upros_message/SingleServo.h"
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

    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_hor_service_close");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    ros::Publisher single_joint_pub = nh.advertise<upros_message::SingleServo>("/single_servo_topic", 10);

    //第一步，运动到倾倒点
    upros_message::ArmPosition move_srv;
    move_srv.request.x = -200.0;
    move_srv.request.y = 50.0;
    move_srv.request.z = 170.0;
    arm_move_close_client.call(move_srv);
    sleep(5.0);

    //第二步，5号舵机自旋180度倒下
    upros_message::SingleServo single_servo;
    single_servo.ID = 5;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = -1480;
    single_joint_pub.publish(single_servo);
    sleep(5.0);

    //第三步，5号舵机转回
    single_servo.Target_position_Angle = 0;
    single_joint_pub.publish(single_servo);
    sleep(5.0);

    //第四步，运动到容器分拣区域
    move_srv.request.x = 200.0;
    move_srv.request.y = 0.0;
    move_srv.request.z = 200.0;
    arm_move_close_client.call(move_srv);
    sleep(5.0);

    //第五步，打开夹爪，容器分拣 
    std_srvs::Empty empty_srv;
    arm_release_client.call(empty_srv);
    sleep(5.0);

    //第六步，返回零位
    arm_zero_client.call(empty_srv);
    sleep(5.0);

    ros::shutdown();

    return 0;
}

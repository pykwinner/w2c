#include "upros_message/SingleServo.h"
#include "upros_message/MultipleServo.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>

ros::Publisher multiple_joint_pub;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

bool up_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // 顶升抬起
    upros_message::MultipleServo multiple_servo;

    upros_message::SingleServo single_servo;
    single_servo.ID = 1;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = 166; 
    multiple_servo.servo_gather.push_back(single_servo);   

    single_servo.ID = 2;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = -205; 
    multiple_servo.servo_gather.push_back(single_servo);

    single_servo.ID = 3;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = 114; 
    multiple_servo.servo_gather.push_back(single_servo); 

    single_servo.ID = 4;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = -123; 
    multiple_servo.servo_gather.push_back(single_servo);
    
    multiple_joint_pub.publish(multiple_servo);
    return true;
}

bool down_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // 顶升落下
    upros_message::MultipleServo multiple_servo;

    upros_message::SingleServo single_servo;
    single_servo.ID = 1;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = 136; 
    multiple_servo.servo_gather.push_back(single_servo);   

    single_servo.ID = 2;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = -178; 
    multiple_servo.servo_gather.push_back(single_servo);

    single_servo.ID = 3;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = 31; 
    multiple_servo.servo_gather.push_back(single_servo); 

    single_servo.ID = 4;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = -37; 
    multiple_servo.servo_gather.push_back(single_servo);
    
    multiple_joint_pub.publish(multiple_servo);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lift_control");

    ros::NodeHandle nh;

    ros::ServiceServer up_service = nh.advertiseService("up_service", up_callback);

    ros::ServiceServer down_service = nh.advertiseService("down_service", down_callback);
    
    multiple_joint_pub = nh.advertise<upros_message::MultipleServo>("/multiple_servo_topic", 10);

    sleep(1.0);

    ros::spin();

    return 0;
}

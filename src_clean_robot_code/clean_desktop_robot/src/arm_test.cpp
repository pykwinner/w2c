#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mgrab_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_close");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    std_srvs::Empty empty_srv;

    // 第一步，打开夹爪
    arm_release_client.call(empty_srv);
    sleep(5.0);

    // 第二步，运动到抓取位置
    upros_message::ArmPosition move_srv;
    move_srv.request.x = 0;
    move_srv.request.y = 230;
    move_srv.request.z = -90;
    arm_move_open_client.call(move_srv);
    sleep(5.0);

    // 第三步，闭合夹爪
    arm_grab_client.call(empty_srv);
    sleep(5.0);

    // 第四步，返回零位
    arm_zero_client.call(empty_srv);
    sleep(5.0);

    // 先前进一小段,大约10cm
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.1;
    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok() && count < 40)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    // 第一步，运动到放置点
    move_srv.request.x = 0;
    move_srv.request.y = 300.0;
    move_srv.request.z = 200.0;
    arm_move_close_client.call(move_srv);
    sleep(5.0);

    // 第二步，打开夹爪
    arm_release_client.call(empty_srv);
    sleep(5.0);

    // 第三步，返回零位
    arm_zero_client.call(empty_srv);
    sleep(5.0);

    vel_msg.linear.x = -0.1;
    count = 0;

    while (ros::ok() && count < 40)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    ros::shutdown();

    return 0;
}

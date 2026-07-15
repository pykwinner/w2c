#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"

#include <string>
#include <stdlib.h>
#include <sensor_msgs/Range.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

float last_range = 10.0;
float target_grab_distance;
bool enable_grab = false;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

void rangeCallback1(const sensor_msgs::Range::ConstPtr &msg)
{
    // 订阅前方tof数值
    ROS_INFO("Distance Front: %f", msg->range);
    float range = msg->range;

    if (last_range >= 0.3 && range < 0.3)
    {
        // 发生突变，可以抓取
        target_grab_distance = range + 0.15;
        enable_grab = true;
    }
    last_range = range;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "w4a_arm_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    // 初始化service创建服务客户端
    ros::ServiceClient arm_pose_client = nh.serviceClient<upros_message::ArmPosition>("/control_center/arm_pos_service");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/control_center/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/control_center/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/control_center/release_service");

    ros::Subscriber sub_1 = nh.subscribe<sensor_msgs::Range>("/us/tof1", 10, rangeCallback1);

    sleep(3.0);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if(enable_grab)
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    // 抓取前单位转换
    float x = 0.0;
    float y = target_grab_distance * 100.0;
    float z = -1.0;

    ROS_INFO("Find Tag x: %f, y: %f, z: %f ;", x, y, z);

    std_srvs::Empty empty_srv;
    sleep(2.0);

    // 逆运算移动抓取到上方
    upros_message::ArmPosition srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z + 5.0;
    arm_pose_client.call(srv);
    sleep(3.0);

    // 下探
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    arm_pose_client.call(srv);
    sleep(3.0);

    // 吸气
    arm_grab_client.call(empty_srv);
    sleep(2.0);

    // 抬起来
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z + 5.0;
    arm_pose_client.call(srv);
    sleep(5.0);

    // 运动放置-高点
    srv.request.x = 19.0;
    srv.request.y = 0.0;
    srv.request.z = z + 5.0;
    arm_pose_client.call(srv);
    sleep(5.0);

    // 运动放置-低点
    srv.request.x = 19.0;
    srv.request.y = 0.0;
    srv.request.z = -1.0;
    arm_pose_client.call(srv);
    sleep(3.0);

    // 呼气
    arm_release_client.call(empty_srv);
    sleep(1.0);

    // 回到零位
    arm_zero_client.call(empty_srv);
    sleep(3.0);

    ros::shutdown();

    return 0;
}
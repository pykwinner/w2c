#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

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
    ros::NodeHandle private_nh("~");
    
    // 在参数中加载要抓取的tag目标
    std::string tag_link;
    private_nh.getParam("tag", tag_link);
    ROS_INFO("The value of tag is %s", tag_link.c_str());

    // 初始化service创建服务客户端
    ros::ServiceClient arm_pose_client = nh.serviceClient<upros_message::ArmPosition>("/control_center/arm_pos_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/control_center/grab_service");

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ROS_INFO("tf coordinate transformaing....");

    // 获取tag到机械臂基坐标的坐标变换
    geometry_msgs::TransformStamped tfs_1 = buffer.lookupTransform("arm_base_link", tag_link, ros::Time(0), ros::Duration(100));

    int bias_x = 0;  //x方向的偏移，增加的机械臂往左多探的毫米数
    int bias_y = 80; //y方向的偏移，增加的机械臂往前多探的毫米数
    int bias_z = 45; //z方向的偏移，增加的机械臂往上多探的毫米数

    // 单位转换，ros坐标系到逆运算坐标系
    int x = int(tfs_1.transform.translation.y * 1000.0) + bias_x; 
    int y = int(tfs_1.transform.translation.x  * 1000.0) + bias_y;
    int z = int(tfs_1.transform.translation.z * 1000.0) + bias_z;

    std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;

    std_srvs::Empty empty_srv;
    
    // 逆运算移动抓取到上方
    upros_message::ArmPosition srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z + 50;
    arm_pose_client.call(srv);
    sleep(3.0);
 
    //下探
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    arm_pose_client.call(srv);
    sleep(3.0);

    // 吸气
    arm_grab_client.call(empty_srv);
    sleep(2.0);

    //抬起来
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z + 120;
    arm_pose_client.call(srv);
    sleep(5.0);

    ros::shutdown();

    return 0;
}

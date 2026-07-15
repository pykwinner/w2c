#include "upros_arm/control_center.h"

// 初始化
void ControlCenter::initROSModule()
{

    // 初始化打开夹爪运动service
    arm_pos_open_claw_service = nn.advertiseService("arm_pos_service_open", &ControlCenter::arm_position_open_claw_callback, this);

    // 初始化闭合夹爪运动service
    arm_pos_close_claw_service = nn.advertiseService("arm_pos_service_close", &ControlCenter::arm_position_close_claw_callback, this);

    // 初始化抓取service
    grab_service = nn.advertiseService("grab_service", &ControlCenter::grab_callback, this);

    // 初始化放松service
    release_service = nn.advertiseService("release_service", &ControlCenter::loose_callback, this);

    // 初始化归零service
    zero_service = nn.advertiseService("zero_service", &ControlCenter::zero_callback, this);

}

//打开夹爪机械臂运动请求：传入位置参数，单位cm，左x正，前y正，上z正，控制机械臂运动，如果 resp.status 返回0，表示超出机械臂运动范围，返回1表示可以运动
bool ControlCenter::arm_position_open_claw_callback(upros_message::ArmPosition::Request &req, upros_message::ArmPosition::Response &resp)
{
    float x = req.x;
    float y = req.y;
    float z = req.z;
    ROS_INFO("arm position controller :x = %.2f, y = %.2f,z = %.2f", x, y, z);
    upros_arm.inverseFind(x, y, z);
    upros_arm.inverseMoveToGrab();
    return true;
}

//闭合夹爪机械臂运动请求：传入位置参数，单位cm，左x正，前y正，上z正，控制机械臂运动，如果 resp.status 返回0，表示超出机械臂运动范围，返回1表示可以运动
bool ControlCenter::arm_position_close_claw_callback(upros_message::ArmPosition::Request &req, upros_message::ArmPosition::Response &resp)
{
    float x = req.x;
    float y = req.y;
    float z = req.z;
    ROS_INFO("arm position controller :x = %.2f, y = %.2f,z = %.2f", x, y, z);
    upros_arm.inverseFind(x, y, z);
    upros_arm.inverseMoveToPut();
    return true;
}

bool ControlCenter::grab_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // 闭合夹爪
    upros_arm.claw_close();
    return true;
}

bool ControlCenter::loose_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // 打开夹爪
    upros_arm.claw_open();
    return true;
}

bool ControlCenter::zero_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // 位置归零
    upros_arm.go_home();
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_center");
    ros::NodeHandle nh("~");
    ControlCenter control_center(nh);
    control_center.initROSModule();
    ros::spin();
    return 0;
}

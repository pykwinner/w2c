#include "control_center.h"

// 初始化
void ControlCenter::initROSModule()
{
    // 初始化机械臂逆运算service
    armPos_service = nn.advertiseService("arm_pos_service", &ControlCenter::arm_position_callback, this);

    // 初始化抓取service
    grab_service = nn.advertiseService("grab_service", &ControlCenter::grab_callback, this);

    // 初始化放松service
    release_service = nn.advertiseService("release_service", &ControlCenter::loose_callback, this);

    // 初始化归零service
    zero_service = nn.advertiseService("zero_service", &ControlCenter::zero_callback, this);

    // 机械臂自身初始化,先归零，再吸放
    ROS_INFO("Init W4A Arm!!!!");
    char portname[20];
    sprintf(portname, "/dev/arm");
    arm.init(portname, 115200);
}

// 请求：传入位置参数，单位cm，左x正，前y正，上z正，控制机械臂运动，如果 resp.status 返回0，表示超出机械臂运动范围，返回1表示可以运动
bool ControlCenter::arm_position_callback(upros_message::ArmPosition::Request &req, upros_message::ArmPosition::Response &resp)
{
    float x = req.x;
    float y = req.y;
    float z = req.z;
    ROS_INFO("arm position controller :x = %.2f, y = %.2f,z = %.2f", x, y, z);
    // 机械臂解算
    vector<int> pos = arm_inverse.get_steps_from_absolute_pos(x, y, z);
    ROS_INFO("setps = %d, %d,%d", pos[0], pos[1], pos[2]);
    int p[3] = {pos[0], pos[1], pos[2]};
    // 判断控制位置都为0
    bool is_allZero = std::all_of(
        std::begin(p),
        std::end(p),
        [](uint32_t item)
        { return item == 0; });
    // 如果xyz坐标不为0的情况下，返回位置解算为0，则表示不在机械臂的工作范围内
    if (is_allZero && (x != 0 || y != 0 || z != 0))
    {
        resp.status = 0;
    }
    else
    {
        // 正常控制机械臂到达当前位置
        arm.armSetAbsSteps(p);
        resp.status = 1;
    }
    return true;
}

bool ControlCenter::grab_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // 开启气泵
    arm.armSetPump(true);
    ros::Duration(1.0).sleep();
    arm.armSetValve(false);
    return true;
}

bool ControlCenter::loose_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    // 关闭气泵
    arm.armSetPump(false);
    ros::Duration(1.0).sleep();
    arm.armSetValve(true);
    return true;
}

bool ControlCenter::zero_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    arm.armSetZeroCal();
    ros::Duration(1.0).sleep();
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

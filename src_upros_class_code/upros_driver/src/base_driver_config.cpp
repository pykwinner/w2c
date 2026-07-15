#include "upros_driver/base_driver_config.h"

#define PI 3.1415926f

BaseDriverConfig::BaseDriverConfig(ros::NodeHandle &p) : pn(p)
{
}

BaseDriverConfig::~BaseDriverConfig()
{
}

void BaseDriverConfig::init()
{
  // 串口号与波特率
  pn.param<std::string>("port", port, "/dev/zoo");
  pn.param<int>("buadrate", buadrate, 115200);

  // 电机码盘分辨率与减速比
  pn.param<float>("motor_encoder", motor_encoder, 64.0);
  pn.param<float>("motor_ratio", motor_ratio, 90.0);

  // 差分轮径和轮距参数
  pn.param<float>("diff_wheel_radius", diff_wheel_radius, 0.099);
  pn.param<float>("diff_wheel_track", diff_wheel_track, 0.216);

  // 全向轮径和轮距参数
  pn.param<float>("onmi_wheel_radius", onmi_wheel_radius, 0.075);
  pn.param<float>("onmi_wheel_track", onmi_wheel_track, 0.1466);

  // 麦轮轮径和轮距参数
  pn.param<float>("mec_wheel_radius", mec_wheel_radius, 0.075);
  pn.param<float>("mec_wheel_track", mec_wheel_track, 0.1466);

  ROS_INFO("port:%s buadrate:%d", port.c_str(), buadrate);
  ROS_INFO("motor_encoder:%f motor_ratio:%f", motor_encoder, motor_ratio);

  // 底盘形态
  pn.param<int>("chassis_type", chassis_type, 1);

  // 里程计坐标系和机体坐标系
  pn.param<std::string>("base_frame", base_frame, "base_footprint");
  pn.param<std::string>("odom_frame", odom_frame, "odom");

  // 是否发布坐标变换，一般不发
  pn.param<bool>("publish_tf", publish_tf, true);

  // 消息名称
  pn.param<std::string>("imu_topic", imu_topic, "handsfree/imu");
  pn.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
  pn.param<std::string>("odom_topic", odom_topic, "odom");
  pn.param<std::string>("cmd_single_servo_topic", cmd_single_servo_topic, "single_servo_topic");
  pn.param<std::string>("cmd_multiple_servo_topic", cmd_multiple_servo_topic, "multiple_servo_topic");

  // 是否发布机械臂当前关节
  pn.param<bool>("publish_joint_state", publish_joint_state, true);

  // 机械臂各关节名称
  pn.param<int>("joint1", joint1, 1);
  pn.param<int>("joint2", joint2, 2);
  pn.param<int>("joint3", joint3, 3);
  pn.param<int>("joint4", joint4, 4);
  pn.param<int>("joint5", joint5, 5);
  pn.param<int>("claw_joint", claw_joint, 6);

  // 机械臂关节发布的主题名称
  pn.param<std::string>("joint_state_topic", joint_state_topic, "/joint_state");
}
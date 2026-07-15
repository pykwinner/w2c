#include "base_driver_config.h"

#include "data_holder.h"
#define PI 3.1415926f

BaseDriverConfig::BaseDriverConfig(ros::NodeHandle &p) : pn(p)
{
}

BaseDriverConfig::~BaseDriverConfig()
{
}

void BaseDriverConfig::init()
{
  // comm param
  pn.param<std::string>("port", port, "/dev/ttyACM0");
  pn.param<int>("buadrate", buadrate, 115200);
  pn.param<float>("motor_encoder", motor_encoder, 64.0);
  pn.param<float>("motor_ratio", motor_ratio, 90.0);
  pn.param<float>("diff_wheel_radius", diff_wheel_radius, 0.099);
  pn.param<float>("diff_wheel_track", diff_wheel_track, 0.216);
  pn.param<float>("onmi_wheel_radius", onmi_wheel_radius, 0.075);
  pn.param<float>("onmi_wheel_track", onmi_wheel_track, 0.1466);
  pn.param<float>("mec_wheel_radius", mec_wheel_radius, 0.075);
  pn.param<float>("mec_wheel_track", mec_wheel_track, 0.1466);

  ROS_INFO("port:%s buadrate:%d", port.c_str(), buadrate);
  ROS_INFO("motor_encoder:%d motor_ratio:%d", motor_encoder, motor_ratio);
  ROS_INFO("diff_wheel_radius:%d diff_wheel_track:%d", diff_wheel_radius, diff_wheel_track);

  pn.param<int>("chassis_type", chassis_type, 1);
  pn.param<std::string>("base_frame", base_frame, "base_link");
  pn.param<std::string>("odom_frame", odom_frame, "odom");
  pn.param<bool>("publish_tf", publish_tf, true);

  // topic name param
  pn.param<std::string>("imu_topic", imu_topic, "handsfree/imu");
  pn.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
  pn.param<std::string>("odom_topic", odom_topic, "odom");
  pn.param<std::string>("cmd_single_servo_topic", cmd_single_servo_topic, "single_servo_topic");
  pn.param<std::string>("cmd_multiple_servo_topic", cmd_multiple_servo_topic, "multiple_servo_topic");

  pn.param<bool>("publish_joint_state", publish_joint_state, true);
  pn.param<int>("joint1", joint1, 1);
  pn.param<int>("joint2", joint2, 2);
  pn.param<int>("joint3", joint3, 3);
  pn.param<int>("joint4", joint4, 4);
  pn.param<int>("joint5", joint5, 5);
  pn.param<int>("claw_joint", claw_joint, 6);
  pn.param<std::string>("joint_state_topic", joint_state_topic, "/s2a/joint_state");
}
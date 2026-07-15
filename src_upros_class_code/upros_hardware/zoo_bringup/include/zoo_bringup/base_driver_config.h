
#include <ros/ros.h>

class Robot_parameter;
class BaseDriverConfig
{
public:
  BaseDriverConfig(ros::NodeHandle &p);
  ~BaseDriverConfig();

  void init();

public:
  std::string port;
  int32_t buadrate;

  int chassis_type;
  std::string base_frame;
  std::string odom_frame;

  bool publish_tf;

  float motor_encoder;
  float motor_ratio;
  float diff_wheel_radius;
  float diff_wheel_track;
  float onmi_wheel_radius;
  float onmi_wheel_track;
  float mec_wheel_radius;
  float mec_wheel_track;

  bool publish_joint_state;
  int joint1;
  int joint2;
  int joint3;
  int joint4;
  int joint5;
  int claw_joint;

  std::string imu_topic;
  std::string cmd_vel_topic;
  std::string cmd_single_servo_topic;
  std::string cmd_multiple_servo_topic;
  std::string odom_topic;
  std::string joint_state_topic;
  ros::NodeHandle &pn;
  ros::ServiceClient client;
  bool set_flag;
};

#ifndef UPROS_DRIVER_H
#define UPROS_DRIVER_H

#include <ros/ros.h>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>

#include "dataframe.h"
#include "base_driver_config.h"
#include "data_holder.h"


#include "upros_message/SingleServo.h"
#include "upros_message/MultipleServo.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16MultiArray.h"
#include "tf/transform_broadcaster.h"

class UprosDriver
{
private:
    UprosDriver();

public:
    // 与下位机通信驱动采用单例类
    static UprosDriver *Instance()
    {
        if (instance == NULL)
        {
            instance = new UprosDriver();
        }
        return instance;
    }

    ~UprosDriver();

    void work_loop();

private:
    void init_topic();
    void init_chassis();    // 设置底盘形态
    void init_sensor();     // 初始化传感器信息
    void get_upros_datas(); // 获取底盘各项信息

    void get_arm_angles(); // 获取机械臂关节角度

    void update_speed(float x, float y, float yaw); // 下发速度控制指令

    void cmd_vel_callback(const geometry_msgs::Twist &vel_cmd);                      // 订阅Topic，下发速度控制命令
    void cmd_single_servo_callback(const upros_message::SingleServo &servoData);     // 订阅Topic，下发单个舵机的控制
    void cmd_multiple_servo_callback(const upros_message::MultipleServo &servoData); // 订阅Topic，下发机械臂整体的舵机控制
    void imu_data_callback(const sensor_msgs::Imu &imu_data);                        // 订阅IMU消息，校准里程计用

    void init_joint();        // 初始化机械臂各关节角度
    void update_joint_info(); // 更新机械臂各个关节角度

    void update_diff_odom(int32 motor1_encoder, int32 motor2_encoder);                                            // 计算差分底盘里程计，适用W2A，W2U
    void update_onmi_odom(int32 motor1_encoder, int32 motor2_encoder, int32 motor3_encoder);                      // 计算全向底盘里程计， 适用W3S
    void update_mec_odom(int32 motor1_encoder, int32 motor2_encoder, int32 motor3_encoder, int32 motor4_encoder); // 计算麦轮底盘里程计，适用W4A，W4A-T

    void convertToUnsignedCharArray(const std::vector<int16_t> &pos, unsigned char params[]); // 转化函数,uint16转2个uint8
    void packServoData(uint8_t id, int16_t angle, uint16_t speed, unsigned char params[]); // 转化函数，舵机ID速度位置转化成数组

public:
    BaseDriverConfig &getBaseDriverConfig()
    {
        return bdg;
    }

    ros::NodeHandle *getNodeHandle()
    {
        return &nh;
    }

    ros::NodeHandle *getPrivateNodeHandle()
    {
        return &pn;
    }

private:

    ros::NodeHandle nh;
    ros::NodeHandle pn;

    // 单例
    static UprosDriver *instance;

    // 配置文件读取
    BaseDriverConfig bdg;

    // 数据封装
    boost::shared_ptr<DataFrame> frame;

    // 底盘控制topic接收
    ros::Subscriber cmd_vel_sub;

    // 超声发布
    ros::Publisher ul_sensor_pub1, ul_sensor_pub2, ul_sensor_pub3, ul_sensor_pub4;

    // tof发布
    ros::Publisher tof_pub1, tof_pub2, tof_pub3, tof_pub4;

    // 碰撞传感器发布
    ros::Publisher bump_sensor_pub;

    // 超声传感器消息
    sensor_msgs::Range ul_sensor1, ul_sensor2, ul_sensor3, ul_sensor4;

    // tof传感器消息
    sensor_msgs::Range tof1, tof2, tof3, tof4;

    // 碰撞传感器消息
    std_msgs::Int16MultiArray bump_sensor_array;

    // 舵机原始位置消息
    std_msgs::Int16MultiArray servo_pos_array;

    // imu数据接收
    ros::Subscriber imu_sub;

    // 里程计累加
    float pos_x;
    float pos_y;
    // 电机编码器最后一帧数据（两轮）
    float last_left_encoder;
    float last_right_encoder;
    // 电机编码器最后一帧数据（三轮、四轮）
    float last_encoder1;
    float last_encoder2;
    float last_encoder3;
    float last_encoder4;

    // imu最后一帧航向角
    float last_yaw;
    // imu当前航向角
    float current_yaw;
    // imu零偏误差
    float first_error_yaw;

    ros::Time last_time;

    bool is_first_measurement;

    // 机械臂的上一帧（用于计算关节速度）
    float last_joint1_pos;
    float last_joint2_pos;
    float last_joint3_pos;
    float last_joint4_pos;
    float last_joint5_pos;
    float last_claw_pos;
    ros::Time last_joint_time;

    // 里程计发布
    ros::Publisher odom_pub;

    // 机械臂位置信息发布
    ros::Publisher joint_pub;
    ros::Publisher servo_pose_pub;

    // 舵机控制消息
    ros::Subscriber cmd_single_servo_sub;
    ros::Subscriber cmd_multiple_servo_sub;

    // 里程计msg
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;

    // 机械臂位置信息
    sensor_msgs::JointState joint_info;

};

#endif
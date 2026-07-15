#ifndef DATA_HOLDER_H_
#define DATA_HOLDER_H_

#include <string.h>
#include <stdint.h>
#include <vector>
using namespace std;

#pragma pack(1)

typedef int int32;
typedef short int16;
typedef unsigned short uint16;

struct Robot_clear_odom
{
    int8_t clear;
};

struct Robot_chassis_type
{
    int8_t type; // 1 四轮全向底盘  2 三轮全向底盘  3 四轮差速底盘
};

struct Robot_velocity
{
    short v_liner_x;   // 线速度 前>0 mm/s
    short v_liner_y;   // 差分轮 为0  mm/s
    short v_angular_z; // 角速度 左>0 0.01rad/s  100 means 1 rad/s
};

struct Robot_odom
{
    short v_liner_x;   // 线速度 前>0 后<0  cm/s
    short v_liner_y;   // 差分轮 为0        cm/s
    short v_angular_z; // 角速度 左>0 右<0  0.01rad/s   100 means 1 rad/s
    int32 x;           // 里程计坐标x       cm (这里long为4字节，下同)
    int32 y;           // 里程计坐标y       cm
    short yaw;         // 里程计航角        0.01rad     100 means 1 rad
};

// 单个舵机信息
struct Single_Servo
{
    uint8_t ID;                    // 舵机ID
    int16_t Target_position_Angle; // 舵机角度
    uint16_t Rotation_Speed;       // 舵机速度
};

// 多个舵机信息
struct Multiple_Servo
{
    Single_Servo servo_gather[20];
};

struct Ul_Sensor
{
    uint16_t tof1; // TOF1 距离
    uint16_t ul1;  // 超声 1 距离
    uint16_t tof2; // TOF2 距离
    uint16_t ul2;  // 超声 2 距离
    uint16_t tof3; // TOF3 距离
    uint16_t ul3;  // 超声 3 距离
    uint16_t tof4; // TOF4 距离
    uint16_t ul4;  // 超声 4 距离
};

struct Sensor_Status
{
    uint16_t adc1;     // ADC1 的值
    uint16_t adc2;     // ADC2 的值
    uint16_t adc3;     // ADC3 的值
    uint16_t adc4;     // ADC4 的值
    uint16_t adc5;     // ADC5 的值
    uint16_t adc6;     // ADC6 的值
    uint16_t adc7;     // ADC7 的值
    uint16_t adc8;     // ADC8 的值
    uint16_t adc9;     // ADC9 的值
    uint16_t adc10;    // ADC10 的值
    uint16_t adc11;    // ADC11 的值
    uint16_t adc12;    // ADC12 的值
    uint8_t collision; // 碰撞传感器信息 bit0-bit3
};

// 电机编码器数据
struct Motor_Encoder
{
    int32_t motor1;
    int32_t motor2;
    int32_t motor3;
    int32_t motor4;
};

struct Servo_Pos
{
    int32_t servo_pos[6];
};

struct Servo_Ids
{
    uint8_t joint1;
    uint8_t joint2;
    uint8_t joint3;
    uint8_t joint4;
    uint8_t joint5;
    uint8_t claw_joint;
};

#pragma pack(0)

class Data_holder
{
public:
    static Data_holder *get()
    {
        static Data_holder dh;
        return &dh;
    }

    void load_parameter();

    void save_parameter();

private:
    Data_holder()
    {
        memset(&clear_odom, 0, sizeof(struct Robot_clear_odom));
        memset(&chassiss_type, 0, sizeof(struct Robot_chassis_type));
        memset(&velocity, 0, sizeof(struct Robot_velocity));
        memset(&odom, 0, sizeof(struct Robot_odom));
        memset(&single_servo, 0, sizeof(Single_Servo));
        memset(&multiple_servo, 0, sizeof(Multiple_Servo));
        memset(&ul_sensor, 0, sizeof(Ul_Sensor));
        memset(&sensor_status, 0, sizeof(Sensor_Status));
        memset(&encoder, 0, sizeof(Motor_Encoder));
        memset(&servo_pos, 0, sizeof(Servo_Pos));
        memset(&servo_ids, 0, sizeof(Servo_Ids));
    }

public:
    struct Robot_clear_odom clear_odom;
    struct Robot_chassis_type chassiss_type;
    struct Robot_velocity velocity;
    struct Robot_odom odom;
    struct Single_Servo single_servo;
    struct Multiple_Servo multiple_servo;
    struct Ul_Sensor ul_sensor;
    struct Sensor_Status sensor_status;
    struct Motor_Encoder encoder;
    struct Servo_Pos servo_pos;
    struct Servo_Ids servo_ids;
};
#endif

#ifndef DATA_HOLDER_H_
#define DATA_HOLDER_H_

#include <string.h>
#include <stdint.h>
#include <vector>
using namespace std;

#pragma pack(1)

// 嵌入式控制器底盘信息
struct Sensor_Status{
    uint16_t tof_1_range; //TOF1数值
    uint16_t sonar_1_range; //超声1数值
    uint16_t tof_2_range; //TOF2数值
    uint16_t sonar_2_range; //超声2数值
    uint16_t tof_3_range; // TOF3数值
    uint16_t sonar_3_range; // 超声3数值
    uint16_t tof_4_range; // TOF4数值
    uint16_t sonar_4_range; // 超声4数值

    uint16_t adc1; //ADC1 的值
    uint16_t adc2; //ADC2 的值
    uint16_t adc3; //ADC3 的值
    uint16_t adc4; //ADC4 的值
    uint16_t adc5; //ADC5 的值
    uint16_t adc6; //ADC6 的值
    uint16_t adc7; //ADC7 的值
    uint16_t adc8; //ADC8 的值
    uint16_t adc9; //ADC9 的值
    uint16_t adc10; //ADC10 的值
    uint16_t adc11; //ADC11 的值
    uint16_t adc12; //ADC12 的值
    uint8_t collision; //碰撞传感器信息 bit0-bit3

    int32_t motor1; //编码器1数值
    int32_t motor2; //编码器2数值
    int32_t motor3; //编码器3数值
    int32_t motor4; //编码器4数值

};

// 5个舵机实际位置
struct Servo_Pos{
    int32_t servo_pos[5];
};

#pragma pack(0)

class Data_holder{
    public:
        static Data_holder* get(){
            static Data_holder dh;
            return &dh;
        }

        void load_parameter();

        void save_parameter();
    
    private:
        Data_holder(){
            memset(&servo_pos, 0, sizeof(Servo_Pos));
            memset(&sensor_status, 0, sizeof(Sensor_Status));
        }
    public:
        struct Sensor_Status sensor_status;
        struct Servo_Pos servo_pos;
};
#endif

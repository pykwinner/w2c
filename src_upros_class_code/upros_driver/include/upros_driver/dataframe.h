#ifndef ZOO_SIMPLE_DATAFRAME_H_
#define ZOO_SIMPLE_DATAFRAME_H_

#include <string.h>
#include <vector>
#include <iostream>
#include <inttypes.h>
#include <deque>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

#include "check_calc.h"
#include "msg_queue.h"
#include "serial/serial.h"

static const unsigned short MESSAGE_BUFFER_SIZE = 255;

typedef int int32;
typedef short int16;
typedef unsigned short uint16;
typedef std::vector<uint8_t> Buffer;

// 全部指令的 CMD
enum CMD
{
    CMD_READ_RAM = 0x03,          // 只读ram
    CMD_WRITE_RAM = 0x06,         // 只写ram，数据掉电丢失
    CMD_WRITE_SERVO_POS = 0x04,   // 舵机写位置
    CMD_READ_SERVO_POS = 0x05,    // 舵机读位置
    CMD_READ_RETURN = 0x30,       // 读返回的CMD
    CMD_SERVO_READ_RETURN = 0x50, // 舵机返回的CMD
    CMD_MESSAGE_MAX
};

const u_int16_t Init_Chassis = 3007; // 初始化底盘的内存表地址
const u_int16_t Set_Vel_Addr = 3001; // 设置速度的内存表地址
const u_int16_t Read_Addr = 4000;    // 读取信息的内存表地址
const u_int16_t Servo_Addr = 0;      // 舵机命令的内存表地址（虚拟）

// 字头
#define FIX_HEAD1 0xAA
#define FIX_HEAD2 0xAA

// 字尾
#define FIX_END1 0x55
#define FIX_END2 0x55

/*
每一个消息帧，帧头的结构体
*/
struct Head
{
    unsigned char head1;  // 头部标记,固定值:0xAA
    unsigned char head2;  // 头部标记,固定值:0xAA
    unsigned char id;     // 设备id
    unsigned char length; // 消息体长度
    unsigned char cmd;    // 指令
    unsigned char addL;   // 地址低字节
    unsigned char addH;   // 地址高字节
};

/*
每一个消息帧，帧尾的结构体
*/
struct End
{
    unsigned char end1; // 尾部标记，固定值：0x55
    unsigned char end2; // 尾部标记，固定值：0x55
};

/*
消息帧结构体
*/
struct Message
{
    struct Head head;                        // 消息头
    unsigned char data[MESSAGE_BUFFER_SIZE]; // 消息体数组
    unsigned char check1;                    // CRC校验1
    unsigned char check2;                    // CRC校验2
    unsigned char recv_count;                // 已经接收的字节数
    struct End end;                          // 消息尾

    Message() {}

    // 构造函数，包含设备id，指令，地址，长度，数据
    Message(unsigned char id, unsigned char cmd, uint16 address, unsigned char len, unsigned char *data)
    {
        head.head1 = FIX_HEAD1;
        head.head2 = FIX_HEAD2;
        head.id = id;
        head.length = recv_count = len + 4;
        head.cmd = cmd;
        head.addL = address & 0x00FF;
        head.addH = address >> 8;
        check1 = 0;
        check2 = 0;
        end.end1 = FIX_END1;
        end.end2 = FIX_END2;
        if (data != 0 && len != 0)
        {
            memcpy(this->data, data, len);
        }
        // 临时数组，用于计算crc校验
        uint8_t temp_msgs[len + 5];
        // 给临时数组赋值
        temp_msgs[0] = id;
        temp_msgs[1] = head.length;
        temp_msgs[2] = head.cmd;
        temp_msgs[3] = head.addL;
        temp_msgs[4] = head.addH;
        for (int i = 0; i < len; i++)
        {
            temp_msgs[i + 5] = this->data[i];
        }
        // 计算crc校验
        uint16_t check = crc16_calc(temp_msgs, sizeof(temp_msgs));
        // 获取crc校验结果的高八位低八位
        check1 = check & 0x00FF;
        check2 = check >> 8;
    }
};

/*
若干枚举变量，用于校验反馈的数据
*/
enum RECEIVE_STATE
{
    STATE_RECV_FIX = 0,
    STATE_RECV_FIX2,
    STATE_RECV_DECEIVEID,
    STATE_RECV_LEN,
    STATE_RECV_CMD,
    STATE_RECV_ADD1,
    STATE_RECV_ADD2,
    STATE_RECV_DATA,
    STATE_RECV_CHECKL,
    STATE_RECV_CHECKH,
    STATE_RECV_END1,
    STATE_RECV_END2
};

class DataFrame
{

public:
    DataFrame(); // 构造函数

    ~DataFrame(); // 析构函数，关闭串口

    bool init(std::string port_name, int32_t baudrate); // 初始化函数，配置并打开串口

    void start_thread();     // 启动线程函数

private:
    //bool recv_proc(unsigned char addL, unsigned char addH); // 数据接收函数

    std::string DecIntToHexStr(char c); // 将 char 转化为16进制字符串的函数，调试用

    void recv_thread_func(); // 接收线程主循环

    void send_thread_func(); // 发送线程主循环

    void process_buffer(); //数据提包函数

    bool data_recv(std::vector<uint8_t> data); // 数据校验函数，校验帧头，ID，数据长度，CMD，地址，CRC，帧尾

    bool data_parse(); // 解析函数，将数据解析到结构体内

public:

    bool send_message(const char id, const char cmd, const uint16 address, unsigned char *params, int params_len); // 数据发布函数

private:
    std::shared_ptr<serial::Serial> serialPtr; // 串口通信类

    Message active_rx_msg;    // 解析完成的数据暂存
    RECEIVE_STATE recv_state; // 数据接收状态
    MsgQueue<Buffer> queue;   // 数据发送队列

    std::thread recv_thread; // 接收线程对象
    std::thread send_thread; // 发送线程对象

    std::atomic<bool> keep_running{false}; // 线程退出标志

    std::mutex serial_mutex; // 串口操作互斥锁

    std::deque<uint8_t> buffer_queue; // 串口回调的数据队列
    std::vector<uint8_t> current_frame; // 当前解析的帧数据
};

#endif

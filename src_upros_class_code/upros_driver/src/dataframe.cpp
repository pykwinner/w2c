#include "upros_driver/dataframe.h"
#include "upros_driver/data_holder.h"
#include <stdio.h>
#include <ros/ros.h>

DataFrame::DataFrame()
{
    recv_state = STATE_RECV_FIX;
    keep_running = false; // 初始化线程标志
}

bool DataFrame::init(std::string port_name, int32_t baudrate)
{
    // 初始化一个串口
    serialPtr = std::make_shared<serial::Serial>(port_name, baudrate, serial::Timeout::simpleTimeout(1000));

    // 打开串口
    try
    {
        ROS_INFO("connecting to main board");
        // serialPtr->open();
        ROS_INFO("connected to main board");
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("oops!!! can't connect to main board");
        std::cerr << e.what() << '\n';
        return false;
    }

    return true;
}

DataFrame::~DataFrame()
{

    keep_running = false; // 通知线程退出

    if (recv_thread.joinable())
    {
        recv_thread.join(); // 等待线程结束
    }

    if (send_thread.joinable())
    {
        send_thread.join(); // 等待线程结束
    }

    // 关闭串口
    if (serialPtr->isOpen())
        serialPtr->close();
}

/*
校验数据帧，校验成功的数据帧保存在active_rx_msg中
*/
bool DataFrame::data_recv(std::vector<uint8_t> data)
{
    // 从第一个字节开始校验
    recv_state = STATE_RECV_FIX;
    for (int i = 0; i < data.size(); i++)
    {
        unsigned char c = data[i];

        // 校验帧头head1
        if (recv_state == STATE_RECV_FIX)
        {
            if (c == FIX_HEAD1)
            {
                // 帧头1校验成功，准备校验帧头2
                // ROS_WARN("HEAD 1 PASS!!!!");
                memset(&active_rx_msg, 0, sizeof(active_rx_msg));
                recv_state = STATE_RECV_FIX2;
            }
            else
            {
                // 帧头1校验失败，准备校验帧头1
                recv_state = STATE_RECV_FIX;
            }
            continue;
        }

        // 校验帧头head2
        if (recv_state == STATE_RECV_FIX2)
        {
            if (c == FIX_HEAD2)
            {
                // 帧头2校验成功，准备校验设备id
                // ROS_WARN("HEAD 2 PASS!!!!");
                active_rx_msg.head.head2 = c;
                recv_state = STATE_RECV_DECEIVEID;
            }
            else
            {
                // 帧头2校验失败，准备校验帧头1
                recv_state = STATE_RECV_FIX;
            }
            continue;
        }

        // 校验设备id
        if (recv_state == STATE_RECV_DECEIVEID)
        {
            if (c < 0x05)
            {
                // 设备id校验成功，准备校验数据长度
                // ROS_WARN("HEAD id PASS!!!!");
                active_rx_msg.head.id = c;
                recv_state = STATE_RECV_LEN;
            }
            else
            {
                // 设备id校验失败，准备校验帧头1
                // ROS_ERROR("HEAD id FAILLED!!!!");
                recv_state = STATE_RECV_FIX;
            }
            continue;
        }

        // 校验数据长度
        if (recv_state == STATE_RECV_LEN)
        {
            int len = (int)c;
            if (len + 7 == data.size())
            {
                // 数据长度校验成功，准备校验指令
                // ROS_WARN("HEAD length PASS!!!!");
                active_rx_msg.head.length = c;
                recv_state = STATE_RECV_CMD;
            }
            else
            {
                // 数据长度校验失败，准备校验帧头1
                // ROS_ERROR("HEAD length FAILED, Len: %d, DatSize: %d", len+7, data.size());
                recv_state = STATE_RECV_FIX;
            }
            continue;
        }

        // 校验指令 目前只有返回 2 种
        if (recv_state == STATE_RECV_CMD)
        {
            if (c == CMD_READ_RETURN)
            {
                // 指令校验成功，准备校验flash地址
                // ROS_WARN("HEAD Base CMD PASS!!!!");
                active_rx_msg.head.cmd = c;
                recv_state = STATE_RECV_ADD1;
            }
            else if (c == CMD_SERVO_READ_RETURN)
            {
                // 指令校验成功，准备校验flash地址
                // ROS_WARN("HEAD Servo PASS!!!!");
                active_rx_msg.head.cmd = c;
                recv_state = STATE_RECV_ADD1;                
            }
            else
            {
                recv_state = STATE_RECV_FIX;
                // ROS_ERROR("HEAD CMD FAILLED!!!!");
            }
            continue;
        }

        // 校验flash地址低八位
        if (recv_state == STATE_RECV_ADD1)
        {
            // 如果是返回的数据指令，地址应该是固定的
            if (active_rx_msg.head.cmd == CMD_READ_RETURN)
            {
                if (c == static_cast<unsigned char>(Read_Addr & 0xFF))
                {
                    // flash地址低八位校验成功，准备校验flash地址高八位
                    // ROS_WARN("HEAD Base ADDL PASS!!!!");
                    active_rx_msg.head.addL = c;
                    recv_state = STATE_RECV_ADD2;
                }
                else
                {
                    // ROS_ERROR("HEAD ADDL FAILLED!!!!");
                    recv_state = STATE_RECV_FIX;
                }
            }
            else if(active_rx_msg.head.cmd == CMD_SERVO_READ_RETURN)
            {
                if (c == static_cast<unsigned char>(Servo_Addr & 0xFF))
                {
                    // flash地址低八位校验成功，准备校验flash地址高八位
                    // ROS_WARN("HEAD Servo ADDL PASS!!!!");
                    active_rx_msg.head.addL = c;
                    recv_state = STATE_RECV_ADD2;
                }
                else
                {
                    // ROS_ERROR("HEAD ADDL FAILLED!!!!");
                    recv_state = STATE_RECV_FIX;
                }               
            }
            continue;
        }

        // 校验flassh地址高八位
        if (recv_state == STATE_RECV_ADD2)
        {
            if (active_rx_msg.head.cmd == CMD_READ_RETURN)
            {
                if ((c == static_cast<unsigned char>(Read_Addr >> 8) & 0xFF))
                {
                    // flash地址高八位校验成功，准备校验数据
                    // ROS_WARN("HEAD Base ADDH PASS!!!!");
                    active_rx_msg.head.addL = c;
                    recv_state = STATE_RECV_DATA;
                }
                else
                {
                    // ROS_ERROR("HEAD ADDH FAILLED!!!!");
                    recv_state = STATE_RECV_FIX;
                }
            }
            else if(active_rx_msg.head.cmd == CMD_SERVO_READ_RETURN)
            {
                if ((c == static_cast<unsigned char>(Servo_Addr >> 8) & 0xFF))
                {
                    // flash地址高八位校验成功，准备校验数据
                    // ROS_WARN("HEAD ADDH PASS!!!!");
                    active_rx_msg.head.addL = c;
                    recv_state = STATE_RECV_DATA;
                }
                else
                {
                    // ROS_ERROR("HEAD ADDH FAILLED!!!!");
                    recv_state = STATE_RECV_FIX;
                }                
            }
            continue;
        }

        // 校验数据，复制到接收结构体内
        if (recv_state == STATE_RECV_DATA)
        {
            active_rx_msg.data[active_rx_msg.recv_count++] = c;
            // 数据整个复制过去，校验CRC低八位
            if (active_rx_msg.recv_count >= active_rx_msg.head.length - 4)
            {
                recv_state = STATE_RECV_CHECKL;
            }
            continue;
        }

        // 校验CRC低八位
        if (recv_state == STATE_RECV_CHECKL)
        {
            // CRC低八位校验成功，准备校验CRC高八位
            // ROS_WARN("HEAD CRCL PASS!!!!");
            active_rx_msg.check1 = c;
            recv_state = STATE_RECV_CHECKH;
            continue;
        }

        if (recv_state == STATE_RECV_CHECKH)
        {
            // CRC高八位校验成功，准备校验尾
            // ROS_WARN("HEAD CRCH PASS!!!!");
            active_rx_msg.check2 = c;
            recv_state = STATE_RECV_END1;
            continue;
        }

        // 校验尾1
        if (recv_state == STATE_RECV_END1)
        {
            if (c == FIX_END1)
            {
                // ROS_WARN("END 1 PASS!!!!");
                active_rx_msg.end.end1 = c;
                recv_state = STATE_RECV_END2;
            }
            else
            {
                recv_state = STATE_RECV_FIX;
            }
            continue;
        }

        // 校验尾2
        if (recv_state == STATE_RECV_END2)
        {
            recv_state = STATE_RECV_FIX;
            if (c == FIX_END2)
            {
                // ROS_WARN("END 2 PASS!!!!");
                active_rx_msg.end.end2 = c;
                return true;
            }
        }
    }

    // 校验没通过，返回
    return false;
}

// 从active_rx_msg中解析，分离出想要的数据
bool DataFrame::data_parse()
{
    unsigned char id = active_rx_msg.head.cmd;
    Data_holder *dh = Data_holder::get();

    switch (id)
    {
    case CMD_READ_RETURN:
    {
        int get_data_size = active_rx_msg.head.length - 4;
        // ROS_INFO("Parase base successful!!");
        // unsigned char arr[] = {active_rx_msg.data[41], active_rx_msg.data[42], active_rx_msg.data[43], active_rx_msg.data[44]};
        // std::cout << " " << DecIntToHexStr(active_rx_msg.data[41]).c_str() << " " << DecIntToHexStr(active_rx_msg.data[42]).c_str() << " " << DecIntToHexStr(active_rx_msg.data[43]).c_str() << " " << DecIntToHexStr(active_rx_msg.data[44]).c_str() << std::endl;

        // int32_t motor_1_value;
        // memcpy(&motor_1_value, &arr, sizeof(arr));
        // ROS_INFO("motor1: %d", motor_1_value);

        if (get_data_size == sizeof(dh->sensor_status))
        {
            memcpy(&dh->sensor_status, active_rx_msg.data, get_data_size);
            // int32_t data_holder_1 = dh->sensor_status.motor1;
            // ROS_INFO("Data holder motor1: %d", data_holder_1);
        }
        else
        {
            ROS_ERROR("Sensor status data size mismatch");
            return false;
        }
        break;
    }
    case CMD_SERVO_READ_RETURN:
    { 
        // ROS_INFO("Parase servo successful!!");
        if (active_rx_msg.head.length >= sizeof(dh->servo_pos))
        {
            memcpy(&dh->servo_pos, active_rx_msg.data, sizeof(dh->servo_pos));
        }
        else
        {
            ROS_ERROR("Servo position data size mismatch");
            return false;
        }
        break;
    }
    default:
    {
        // ROS_WARN("Unknown command ID: %02X", id);
        return false; // 未知命令应返回错误
    }
    }
    return true;
}

// 串口数据生成，下发给下位机，数据包含id，cmd，flash地址，数据，crc校验，数据长度
bool DataFrame::send_message(const char id, const char cmd, const uint16 address, unsigned char *params, int params_len)
{
    unsigned char pkg[params_len];
    for (int i = 0; i < params_len; i++)
    {
        pkg[i] = params[i];
    }
    Message msg(id, cmd, address, params_len, pkg);
    // 将消息结构体转换为生成缓存数据包（结构体转数组），按顺序填入data数组中
    Buffer data;
    data.push_back(msg.head.head1);
    data.push_back(msg.head.head2);
    data.push_back(msg.head.id);
    data.push_back(msg.head.length);
    data.push_back(msg.head.cmd);
    data.push_back(msg.head.addL);
    data.push_back(msg.head.addH);
    for (int i = 0; i < params_len; i++)
    {
        data.push_back(params[i]);
    }
    data.push_back(msg.check1);
    data.push_back(msg.check2);
    data.push_back(msg.end.end1);
    data.push_back(msg.end.end2);
    queue.push(data);
    return true;
}

// 启动发送数据包线程
void DataFrame::send_thread_func()
{

    while (keep_running)
    {
        // 如果发送缓存队列不为空，发送数据帧
        if (!queue.isEmpty())
        {
            // 从待发送队列中取出一个数据包，写入串口
            Buffer msg = queue.get_and_pop();
            size_t bytes_written = serialPtr->write(msg.data(), msg.size());
        }
        // 发送缓存区为空
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(10)); // 防止CPU过载
        }
    }
}

void DataFrame::start_thread()
{
    // 串口校验不通过，不启动线程
    if (!serialPtr || !serialPtr->isOpen())
    {
        throw std::runtime_error("Serial port not initialized");
    }

    keep_running = true;

    recv_thread = std::thread(&DataFrame::recv_thread_func, this); // 启动接收线程
    send_thread = std::thread(&DataFrame::send_thread_func, this); // 启动发送线程
}

void DataFrame::recv_thread_func()
{
    const size_t BUFFER_SIZE = 1024;
    std::vector<uint8_t> buffer(BUFFER_SIZE, 0);

    while (keep_running)
    {
        try
        {
            size_t bytes_available = 0;
            {
                std::lock_guard<std::mutex> lock(serial_mutex);
                bytes_available = serialPtr->available();
            }

            // 如果串口缓存区存在可获取数据
            if (bytes_available > 0)
            {
                size_t bytes_read = 0;
                {
                    std::lock_guard<std::mutex> lock(serial_mutex);
                    bytes_read = serialPtr->read(buffer.data(), bytes_available);
                }

                // 如果读取到的数据大字节数于0
                if (bytes_read > 0)
                {
                    // 读取串口缓存区的数据
                    std::vector<uint8_t> received_data(buffer.begin(), buffer.begin() + bytes_read);

                    // 将收到的数据塞入队列
                    buffer_queue.insert(buffer_queue.end(), received_data.begin(), received_data.end());

                    // 处理队列的数据
                    process_buffer();
                }
            }
            // 适当休眠避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        catch (const std::exception &e)
        {
            // 处理异常（例如串口断开）
            std::cerr << "Serial thread error: " << e.what() << std::endl;
            break;
        }
    }
}

void DataFrame::process_buffer()
{
    bool got = false;
    while (true)
    {
        // 1. 寻找帧头 0xAA 0xAA
        size_t start_pos = 0;
        bool found_start = false;
        for (; start_pos + 1 < buffer_queue.size(); ++start_pos)
        {
            if (buffer_queue[start_pos] == FIX_HEAD1 && buffer_queue[start_pos + 1] == FIX_HEAD2)
            {
                found_start = true;
                break;
            }
        }

        // ROS_WARN("start_pos: %d", start_pos);

        // 2. 删除帧头前的无效数据
        buffer_queue.erase(buffer_queue.begin(), buffer_queue.begin() + start_pos);

        // 3. 检查是否有足够数据包含帧尾
        if (buffer_queue.size() < 4)
            return; // 至少需要 AA,AA + EE,EE

        // 4. 寻找帧尾 0xEE 0xEE
        size_t end_pos = 2; // 跳过帧头
        bool found_end = false;
        for (; end_pos + 1 < buffer_queue.size(); ++end_pos)
        {
            if (buffer_queue[end_pos] == FIX_END1 && buffer_queue[end_pos + 1] == FIX_END2)
            {
                found_end = true;
                break;
            }
        }

        // 未找到帧尾，等待更多数据
        if (!found_end)
            return;

        // ROS_WARN("end_pos: %d", end_pos);

        // 5. 提取有效数据（帧头到帧尾巴）
        current_frame.assign(
            buffer_queue.begin(),
            buffer_queue.begin() + end_pos + 2);
        
        //ROS_WARN("current_frame size: %d", current_frame.size());

        // 6. 调用解析逻辑
        if (data_recv(current_frame))
        {
            // 成功解析后，删除整个帧（包括头尾）
            // ROS_WARN("Current frame size: %d", current_frame.size());
            buffer_queue.erase(buffer_queue.begin(), buffer_queue.begin() + end_pos + 2);
            got = true;
        }
        else
        {
            // 解析失败，丢弃该帧
            buffer_queue.erase(buffer_queue.begin(), buffer_queue.begin() + end_pos + 2);
            break;
        }

        // ROS_WARN("buffer_queue size: %d", buffer_queue.size());

        if (got)
        {
            this->data_parse();
            break;
        }
    }
}


// 十进制转十六进制字符串的函数，返回字符串变量，显示调试的时候有用
std::string DataFrame::DecIntToHexStr(char c)
{
    // 处理符号问题：转无符号后再计算
    unsigned char uc = static_cast<unsigned char>(c);
    int decimalNumber = uc;

    std::vector<int> ivec;

    // 特判 0 的情况
    if (decimalNumber == 0)
    {
        return "00";
    }

    // 提取十六进制位（自动逆序）
    while (decimalNumber != 0)
    {
        ivec.push_back(decimalNumber % 16);
        decimalNumber /= 16;
    }

    // 逆序排列得到正确的高低位顺序
    std::reverse(ivec.begin(), ivec.end());

    std::string hexadecimal;
    for (int digit : ivec)
    {
        char ch;
        if (digit > 9)
        {
            ch = 'A' + (digit - 10); // 10→A, 11→B, ..., 15→F
        }
        else
        {
            ch = '0' + digit; // 0→0, 1→1, ..., 9→9
        }
        hexadecimal += ch;
    }

    // 补零到两位（如 "F" → "0F"）
    if (hexadecimal.size() < 2)
    {
        hexadecimal = std::string(2 - hexadecimal.size(), '0') + hexadecimal;
    }

    return hexadecimal;
}
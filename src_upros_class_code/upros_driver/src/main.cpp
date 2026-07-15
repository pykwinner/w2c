#include <ros/ros.h>
#include "upros_driver/upros_driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "upros_driver");
    UprosDriver::Instance()->work_loop();
    ros::spin();
    return 0;
}

// #include <ros/ros.h>
// #include <thread>
// #include <mutex>
// #include "upros_driver/upros_driver.h"
// #include "upros_driver/dataframe.h"

// std::mutex serial_mutex;
// SerialPort serial_port("/dev/zoo", 115200);
// Buffer fixed_buffer = {0xAA, 0xAA, 0x01, 0x05, 0x03, 0xA0, 0x0F, 0x39, 0x08, 0x4E, 0x55, 0x55};

// void writeThread()
// {
//     ros::Rate rate(20); // 100Hz (0.01秒间隔)
//     while (ros::ok())
//     {
//         {
//             std::lock_guard<std::mutex> lock(serial_mutex);
//             serial_port.write(fixed_buffer);
//         }
//         rate.sleep();
//     }
// }

// void readThread()
// {
//     ros::Rate rate(20); // 20Hz
//     while (ros::ok())
//     {
//         Buffer data;
//         {
//             std::lock_guard<std::mutex> lock(serial_mutex);
//             data = serial_port.read();
//         }
//         if (!data.empty())
//         {
//             ROS_WARN("Message RECEIVE Size: %d", data.size());
//         }
//         rate.sleep();
//     }
// }

// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "upros_driver");
//     ros::NodeHandle n;

//     if (!serial_port.init())
//     {
//         ROS_ERROR("Serial port initialization failed");
//         return -1;
//     }
//     ROS_INFO("Serial port initialized successfully");

//     std::thread writer(writeThread);
//     std::thread reader(readThread);

//     // 主线程处理ROS事件
//     ros::spin();

//     // 等待线程结束
//     writer.join();
//     reader.join();

//     return 0;
// }

#include <algorithm>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_srvs/Empty.h>

class ShooterController
{
private:
    typedef std::vector<uint8_t> Buffer;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::ServiceServer shoot_server_;
    ros::ServiceServer close_server_;

    // 使用墙上时间，不受/use_sim_time影响
    ros::WallTimer watchdog_timer_;

    serial::Serial serial_;

    // 原项目使用的单字节控制指令
    Buffer open_buffer_;
    Buffer close_buffer_;

    // 参数
    std::string port_;
    std::string shoot_service_name_;
    std::string close_service_name_;

    int baudrate_;
    int serial_timeout_ms_;

    // 激光允许连续开启的最长时间
    double max_laser_on_time_;

    // 自动关闭失败后的重试间隔
    double close_retry_interval_;

    // 关闭失败多少次后输出严重报警
    int close_failure_report_threshold_;

    // 软件记录的激光状态
    bool laser_on_;

    // 是否已经输出过严重关闭失败报警
    bool close_failure_reported_;

    // 连续关闭失败次数
    int close_retry_count_;

    // 激光打开时间
    ros::WallTime laser_open_time_;

    // 上一次自动关闭尝试时间
    ros::WallTime last_close_attempt_time_;

public:
    ShooterController()
        : private_nh_("~"),
          open_buffer_(1, 0xA3),
          close_buffer_(1, 0xA0),
          port_("/dev/arm"),
          shoot_service_name_("/shoot"),
          close_service_name_("/close"),
          baudrate_(9600),
          serial_timeout_ms_(200),
          max_laser_on_time_(1.0),
          close_retry_interval_(0.20),
          close_failure_report_threshold_(10),
          laser_on_(false),
          close_failure_reported_(false),
          close_retry_count_(0)
    {
        loadParameters();
        openSerial();

        /*
         * 节点启动时先发送一次关闭命令。
         * 即使软件认为激光没有打开，也要主动让硬件关闭。
         */
        if (!closeLaser("startup safety close", true))
        {
            throw std::runtime_error(
                "Failed to send startup laser close command");
        }

        shoot_server_ = nh_.advertiseService(
            shoot_service_name_,
            &ShooterController::shootCallback,
            this);

        close_server_ = nh_.advertiseService(
            close_service_name_,
            &ShooterController::closeCallback,
            this);

        /*
         * 每50毫秒检查一次激光开启时间。
         * WallTimer不会受到ROS仿真时间的影响。
         */
        watchdog_timer_ = nh_.createWallTimer(
            ros::WallDuration(0.05),
            &ShooterController::watchdogCallback,
            this);

        ROS_INFO(
            "Shooter controller ready: "
            "port=%s, baudrate=%d, max_on_time=%.2f s",
            port_.c_str(),
            baudrate_,
            max_laser_on_time_);
    }

    ~ShooterController()
    {
        /*
         * 节点正常退出时，再尝试发送一次关闭命令。
         * 析构函数不能向外抛出异常。
         */
        try
        {
            if (serial_.isOpen())
            {
                closeLaser("node shutdown", false);
                serial_.close();
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR(
                "Exception while closing serial port: %s",
                e.what());
        }
    }

private:
    void loadParameters()
    {
        private_nh_.param<std::string>(
            "port",
            port_,
            "/dev/arm");

        private_nh_.param<std::string>(
            "shoot_service",
            shoot_service_name_,
            "/shoot");

        private_nh_.param<std::string>(
            "close_service",
            close_service_name_,
            "/close");

        private_nh_.param(
            "baudrate",
            baudrate_,
            9600);

        private_nh_.param(
            "serial_timeout_ms",
            serial_timeout_ms_,
            200);

        private_nh_.param(
            "max_laser_on_time",
            max_laser_on_time_,
            1.0);

        private_nh_.param(
            "close_retry_interval",
            close_retry_interval_,
            0.20);

        private_nh_.param(
            "close_failure_report_threshold",
            close_failure_report_threshold_,
            10);

        // 修正错误参数
        baudrate_ =
            std::max(1, baudrate_);

        serial_timeout_ms_ =
            std::max(1, serial_timeout_ms_);

        max_laser_on_time_ =
            std::max(0.10, max_laser_on_time_);

        close_retry_interval_ =
            std::max(0.05, close_retry_interval_);

        close_failure_report_threshold_ =
            std::max(1, close_failure_report_threshold_);
    }

    void openSerial()
    {
        try
        {
            serial_.setPort(port_);

            serial_.setBaudrate(
                static_cast<uint32_t>(baudrate_));

            serial::Timeout timeout =
                serial::Timeout::simpleTimeout(
                    static_cast<uint32_t>(
                        serial_timeout_ms_));

            serial_.setTimeout(timeout);
            serial_.open();
        }
        catch (const std::exception &e)
        {
            ROS_ERROR(
                "Serial initialization failed: %s",
                e.what());

            throw;
        }

        if (!serial_.isOpen())
        {
            throw std::runtime_error(
                "Serial port did not open");
        }

        ROS_INFO(
            "Serial port opened successfully: %s",
            port_.c_str());
    }

    bool writeCommand(
        const Buffer &command,
        const std::string &description,
        bool report_error)
    {
        if (!serial_.isOpen())
        {
            if (report_error)
            {
                ROS_ERROR(
                    "Cannot send %s: serial port is not open",
                    description.c_str());
            }

            return false;
        }

        try
        {
            const size_t written =
                serial_.write(command);

            if (written != command.size())
            {
                if (report_error)
                {
                    ROS_ERROR(
                        "Incomplete serial write for %s: "
                        "expected=%lu, written=%lu",
                        description.c_str(),
                        static_cast<unsigned long>(
                            command.size()),
                        static_cast<unsigned long>(
                            written));
                }

                return false;
            }
        }
        catch (const std::exception &e)
        {
            if (report_error)
            {
                ROS_ERROR(
                    "Failed to send %s: %s",
                    description.c_str(),
                    e.what());
            }

            return false;
        }

        return true;
    }

    bool openLaser()
    {
        /*
         * 正常流程中，每个靶子只调用一次/shoot。
         * 如果激光已经开启，说明控制状态可能异常。
         */
        if (laser_on_)
        {
            ROS_ERROR(
                "Laser is already on; "
                "refusing duplicate shoot command");

            return false;
        }

        if (!writeCommand(
                open_buffer_,
                "laser open command 0xA3",
                true))
        {
            return false;
        }

        laser_on_ = true;
        laser_open_time_ = ros::WallTime::now();

        // 每次重新开启激光都重置关闭重试状态
        last_close_attempt_time_ = ros::WallTime();
        close_retry_count_ = 0;
        close_failure_reported_ = false;

        ROS_INFO(
            "Laser opened: command=0xA3, "
            "automatic close after %.2f seconds",
            max_laser_on_time_);

        return true;
    }

    bool closeLaser(
        const std::string &reason,
        bool report_error)
    {
        /*
         * 即使laser_on_为false，也发送关闭命令。
         * 这样可以重新同步软件状态和硬件状态。
         */
        if (!writeCommand(
                close_buffer_,
                "laser close command 0xA0",
                report_error))
        {
            /*
             * 发送失败时不能设置laser_on_=false。
             * 因为无法确认硬件是否真的关闭。
             */
            return false;
        }

        laser_on_ = false;
        close_retry_count_ = 0;
        close_failure_reported_ = false;

        ROS_INFO(
            "Laser closed: command=0xA0, reason=%s",
            reason.c_str());

        return true;
    }

    bool shootCallback(
        std_srvs::Empty::Request &request,
        std_srvs::Empty::Response &response)
    {
        // Empty服务没有需要处理的请求和响应内容
        (void)request;
        (void)response;

        if (!openLaser())
        {
            ROS_ERROR("Shoot service failed");
            return false;
        }

        return true;
    }

    bool closeCallback(
        std_srvs::Empty::Request &request,
        std_srvs::Empty::Response &response)
    {
        (void)request;
        (void)response;

        if (!closeLaser(
                "close service request",
                true))
        {
            ROS_ERROR("Close service failed");
            return false;
        }

        return true;
    }

    void watchdogCallback(
        const ros::WallTimerEvent &event)
    {
        (void)event;

        if (!laser_on_)
        {
            return;
        }

        const ros::WallTime now =
            ros::WallTime::now();

        const double laser_on_duration =
            (now - laser_open_time_).toSec();

        // 激光尚未超过最长开启时间
        if (laser_on_duration <
            max_laser_on_time_)
        {
            return;
        }

        /*
         * 限制关闭重试频率。
         * 防止以20Hz持续向串口写关闭命令。
         */
        if (!last_close_attempt_time_.isZero())
        {
            const double retry_elapsed =
                (now -
                 last_close_attempt_time_).toSec();

            if (retry_elapsed <
                close_retry_interval_)
            {
                return;
            }
        }

        last_close_attempt_time_ = now;

        /*
         * 这里关闭普通错误输出，由下面的限频日志报告，
         * 避免串口故障时大量刷屏。
         */
        if (closeLaser(
                "automatic safety timeout",
                false))
        {
            return;
        }

        ++close_retry_count_;

        ROS_ERROR_THROTTLE(
            1.0,
            "Automatic laser close failed; retry count=%d",
            close_retry_count_);

        /*
         * 达到阈值后输出一次严重报警。
         * 不能将laser_on_设为false，也不能停止重试，
         * 因为硬件可能仍处于开启状态。
         */
        if (close_retry_count_ >=
                close_failure_report_threshold_ &&
            !close_failure_reported_)
        {
            close_failure_reported_ = true;

            ROS_FATAL(
                "Laser close has failed %d times. "
                "Serial connection or shooter hardware may "
                "be faulty. The controller will continue "
                "trying to close the laser.",
                close_retry_count_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(
        argc,
        argv,
        "shooter_controller");

    try
    {
        ShooterController controller;

        // 使用单线程回调，避免串口同时被多个回调操作
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL(
            "Shooter controller failed to start: %s",
            e.what());

        return 1;
    }

    ROS_INFO("Shooter controller shutdown");

    return 0;
}

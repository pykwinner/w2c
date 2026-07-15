#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

class LaserFilterNode
{
public:
    LaserFilterNode() : nh_("~")
    {
        sub_ = nh_.subscribe("/scan", 1, &LaserFilterNode::laserScanCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        sensor_msgs::LaserScan filtered_scan = *msg;
        const float isolation_threshold = 0.05; // 孤立点判定阈值

        // 第一步：过滤范围外的点
        for (size_t i = 0; i < filtered_scan.ranges.size(); ++i)
        {
            if (filtered_scan.ranges[i] < 0.25 || filtered_scan.ranges[i] > 12.0)
            {
                filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        // 第二步：检测孤立点
        for (size_t i = 0; i < filtered_scan.ranges.size(); ++i)
        {
            if (std::isnan(filtered_scan.ranges[i]))
                continue;

            bool has_neighbor = false;
            // 检查左侧相邻点
            if (i > 0 && !std::isnan(filtered_scan.ranges[i - 1]) &&
                std::abs(filtered_scan.ranges[i] - filtered_scan.ranges[i - 1]) <= isolation_threshold)
            {
                has_neighbor = true;
            }
            // 检查右侧相邻点
            if (i < filtered_scan.ranges.size() - 1 && !std::isnan(filtered_scan.ranges[i + 1]) &&
                std::abs(filtered_scan.ranges[i] - filtered_scan.ranges[i + 1]) <= isolation_threshold)
            {
                has_neighbor = true;
            }

            if (!has_neighbor)
            {
                filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        pub_.publish(filtered_scan);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_filter");
    LaserFilterNode node;
    ros::spin();
    return 0;
}
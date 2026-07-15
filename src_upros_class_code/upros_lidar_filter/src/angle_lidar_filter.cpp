#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class LaserFilterNode {
public:
    LaserFilterNode() : nh_("~") {
        sub_ = nh_.subscribe("/scan", 1, &LaserFilterNode::laserScanCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan filtered_scan;
        filtered_scan.header = msg->header;
        filtered_scan.angle_min = -90 * (3.141592653589793 / 180);
        filtered_scan.angle_max = 90 * (3.141592653589793 / 180);
        filtered_scan.angle_increment = msg->angle_increment;
        filtered_scan.time_increment = msg->time_increment;
        filtered_scan.scan_time = msg->scan_time;
        filtered_scan.range_min = msg->range_min;
        filtered_scan.range_max = msg->range_max;

        int start_index = static_cast<int>((filtered_scan.angle_min - msg->angle_min) / msg->angle_increment);
        int end_index = static_cast<int>((filtered_scan.angle_max - msg->angle_min) / msg->angle_increment);

        for (int i = start_index; i < end_index; ++i) {
            filtered_scan.ranges.push_back(msg->ranges[i]);
        }

        pub_.publish(filtered_scan);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_filter");
    LaserFilterNode node;
    ros::spin();
    return 0;
}
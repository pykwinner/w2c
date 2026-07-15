#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class LaserFilterNode {
public:
LaserFilterNode() : nh_("~") 
{
        sub_ = nh_.subscribe("/scan", 1, &LaserFilterNode::laserScanCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
        sensor_msgs::LaserScan filtered_scan = *msg;
        // 只保留0.25-2.0米之间的数据
        for (unsigned int i = 0; i < filtered_scan.ranges.size(); ++i) {
            if (filtered_scan.ranges[i] < 0.25 || filtered_scan.ranges[i] > 10.0) {
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_filter");
    LaserFilterNode node;
    ros::spin();
    return 0;
}

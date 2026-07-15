#include "ros/ros.h"  
#include "sensor_msgs/LaserScan.h"  
#include "tf2_ros/transform_listener.h"  
#include "geometry_msgs/TransformStamped.h"  
#include <cmath>  
#include <limits>  

class LaserFilterNode {
public:
    LaserFilterNode() : nh_("~") {
        sub_ = nh_.subscribe("/scan", 1, &LaserFilterNode::laserScanCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
        tfBuffer_.reset(new tf2_ros::Buffer());  
        listener_.reset(new tf2_ros::TransformListener(*tfBuffer_)); 
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {

        sensor_msgs::LaserScan filtered_scan = *msg;

        filtered_scan.header.frame_id = "base_link";

        geometry_msgs::TransformStamped transform;  
  
        try {  
            transform = tfBuffer_->lookupTransform("base_link", msg->header.frame_id,  
                                                   ros::Time(0), ros::Duration(0.1));  
        } catch (tf2::TransformException &ex) {  
            ROS_WARN("%s", ex.what());  
            return;  
        }  

        // Convert ranges to base_link frame  
        for (unsigned int i = 0; i < filtered_scan.ranges.size(); ++i) {  

            double angle = msg->angle_min + i * msg->angle_increment;  

            double x = filtered_scan.ranges[i] * cos(angle);  
            double y = filtered_scan.ranges[i] * sin(angle);  
  
            // Rotate and translate to base_link frame  
            double rotated_x = x * transform.transform.rotation.w - y * transform.transform.rotation.z;  
            double rotated_y = x * transform.transform.rotation.z + y * transform.transform.rotation.w;  

            double translated_x = rotated_x + transform.transform.translation.x;  
            double translated_y = rotated_y + transform.transform.translation.y;  
  
            // Calculate new range  
            filtered_scan.ranges[i] = sqrt(translated_x * translated_x + translated_y * translated_y);  
  
            // Filter ranges based on distance and intensity  
            // if (filtered_scan.ranges[i] < 0.25 || filtered_scan.ranges[i] > 15.0 || filtered_scan.intensities[i] < 20.0) {  
            //     filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();  
            // }  
  
            // Filter angles between 40 and 50 degrees  
            if (angle >= 40.0 * M_PI / 180.0 && angle <= 50.0 * M_PI / 180.0) {  
                filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();  
            }  
        } 

        pub_.publish(filtered_scan);
    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;  
    std::unique_ptr<tf2_ros::TransformListener> listener_;  

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_filter");
    LaserFilterNode node;
    ros::spin();
    return 0;
}

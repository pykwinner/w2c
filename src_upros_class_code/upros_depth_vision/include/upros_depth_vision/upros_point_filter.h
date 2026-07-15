#ifndef POINT_FILTER
#define POINT_FILTER

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/transform_listener.h>

using namespace std;


class PointcloudConcatenate
{
public:
    PointcloudConcatenate(ros::NodeHandle &nh);
    ~PointcloudConcatenate();
    void update();

private:
    void subCallbackCloudIn(sensor_msgs::PointCloud2 msg);
    void publishConcatenate(sensor_msgs::PointCloud2 cloud);

    ros::NodeHandle nh_;

    std::string param_frame_target_;

    // 点云的subscriber
    ros::Subscriber sub_cloud_in;

    // 发布的点云
    ros::Publisher pub_concatenate_;

    sensor_msgs::PointCloud2 cloud_in;

    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf::TransformListener *tf_listener;

    void height_filter(sensor_msgs::PointCloud2 &cloud_src,
                       sensor_msgs::PointCloud2 &cloud_out);

    void voxel_filter(sensor_msgs::PointCloud2 &src,
                      sensor_msgs::PointCloud2 &cloud_out);               
};

#endif

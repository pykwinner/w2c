#include "upros_depth_vision/upros_point_filter.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>

using namespace std;

PointcloudConcatenate::PointcloudConcatenate(ros::NodeHandle &nh)
{
    nh_ = nh;
    param_frame_target_ = "base_link";

    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_listener = new tf::TransformListener();

    sub_cloud_in = nh_.subscribe("/camera/depth/points", 1, &PointcloudConcatenate::subCallbackCloudIn, this);
    pub_concatenate_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 10);
}

PointcloudConcatenate::~PointcloudConcatenate()
{
    ROS_INFO("Destructing Pointcloud Concatenate...");
}

// 点云回调函数
void PointcloudConcatenate::subCallbackCloudIn(sensor_msgs::PointCloud2 msg)
{
    cloud_in = msg;
}

void PointcloudConcatenate::update()
{
    sensor_msgs::PointCloud2 cloud_after_transform;
    bool success;
    // 坐标变换到base_link坐标系下
    success = tf_listener->waitForTransform("base_link", cloud_in.header.frame_id, cloud_in.header.stamp, ros::Duration(0.1));
    if (!success)
    {
        ROS_ERROR("Cant get Transform from cloud1 to base_link");
        return;
    }
    success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in, cloud_after_transform, *tf_listener);
    if (!success)
    {
        ROS_WARN("Transforming cloud 1 from %s to %s failed!",
                 cloud_in.header.frame_id.c_str(),
                 param_frame_target_.c_str());
        return;
    }
    
    //高度滤波
    sensor_msgs::PointCloud2 cloud_after_height_filter;
    height_filter(cloud_after_transform, cloud_after_height_filter);
    
    //体素滤波
    sensor_msgs::PointCloud2 cloud_after_voxel_filter;
    voxel_filter(cloud_after_height_filter, cloud_after_voxel_filter);

    publishConcatenate(cloud_after_voxel_filter);
}

// 高度滤波，只保留有效高度内点云
void PointcloudConcatenate::height_filter(sensor_msgs::PointCloud2 &src,
                                          sensor_msgs::PointCloud2 &cloud_out)
{
    cloud_out.data.clear();
    cloud_out.fields = src.fields;
    cloud_out.header = src.header;
    cloud_out.is_bigendian = src.is_bigendian;
    cloud_out.height = 1;
    cloud_out.point_step = src.point_step;

    float min_z = 0.0; // 最低高度-地面
    float max_z = 0.3; // 最高高度-机器人身高

    float max_distance = 1.0; // 使用的最远距离的点云
    float min_distance = 0.2; // 使用的最近距离的点云

    pcl::PointCloud<pcl::PointXYZ>::iterator it;

    float height_threshold = 0.0;

    sensor_msgs::PointCloud2Iterator<float> iter_x(src, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(src, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(src, "z");
    std::vector<unsigned char>::const_iterator iter_global = src.data.begin(), iter_global_end = src.data.end();
    int point_cnt = 0;
    for (; iter_global != iter_global_end; iter_global += cloud_out.point_step, ++iter_x, ++iter_y, ++iter_z)
    {
        float current_point_height = *iter_z;
        float x = *iter_x;
        float y = *iter_y;
        // x方向范围滤波
        if (x < min_distance || x > max_distance)
        {
            continue;
        }
        if (current_point_height < height_threshold)
        {
            continue;
        }
        cloud_out.data.insert(cloud_out.data.end(), iter_global, iter_global + cloud_out.point_step);
        point_cnt++;
    }
    cloud_out.width = point_cnt;
    cloud_out.row_step = cloud_out.width * cloud_out.point_step;
    cloud_out.is_dense = true;
}

// 点云体素化
void PointcloudConcatenate::voxel_filter(sensor_msgs::PointCloud2 &src,
                                         sensor_msgs::PointCloud2 &cloud_out)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_src;
    pcl::fromROSMsg(src, cloud_src);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer(new pcl::PointCloud<pcl::PointXYZ>); // 指针指向cloud_src
    cloud_pointer = cloud_src.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer_filtered(new pcl::PointCloud<pcl::PointXYZ>); // 下采样后的指针
    float leftSize = 0.05f;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_pointer);
    sor.setLeafSize(leftSize, leftSize, leftSize);
    sor.filter(*cloud_pointer_filtered);
    pcl::toROSMsg(*cloud_pointer_filtered, cloud_out);
}

// 发布滤波的点云
void PointcloudConcatenate::publishConcatenate(sensor_msgs::PointCloud2 cloud)
{
    pub_concatenate_.publish(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_concatenate");
    ros::NodeHandle nh;

    PointcloudConcatenate node(nh);

    ROS_INFO("Spinning...");
    ros::Rate rate(10);

    while (ros::ok())
    {
        node.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include "std_srvs/Empty.h"
#include "clean_desktop_robot/params_colorConfig.h"
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

class Color_detect
{
private:
    uint8_t color_select;
    cv::Mat camera_matrix;
    cv::Mat camera_dis;
    uint8_t hue_min_ = 0;
    uint8_t hue_max_ = 255;
    uint8_t saturation_min_ = 56;
    uint8_t saturation_max_ = 255;
    uint8_t value_min_ = 217;
    uint8_t value_max_ = 255;
    bool camera_info = 0;
    float x, y, dis;
    sensor_msgs::Image::ConstPtr result_img;
    ros::Subscriber visual_sub;
    image_transport::Publisher image_result_pub; // 图像发布器
    uint8_t color_hsv[5][6] = {
        //    low          upper
        {0, 0, 0, 0, 0, 0},           // dynamic_hsv
        {61, 115, 40, 74, 255, 255},  // green_hsv
        {80, 90, 100, 130, 200, 255}, // blue_hsv
        {15, 110, 30, 50, 255, 255},  // yellow_hsv
        {0, 100, 53, 18, 223, 255},   // red_hsv
    };

    std::shared_ptr<image_transport::ImageTransport> image_transport_result_ptr_pub; // 图像传输对象指针（用于发布）
    ros::NodeHandle nh;

    std::string rgb_result_pub_topic; // 识别结果图像话题
    geometry_msgs::TransformStamped obg_msg;
    tf2_ros::TransformBroadcaster tf_pub;
    ros::Publisher image_pub;

    message_filters::Subscriber<sensor_msgs::Image> *rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub;

    dynamic_reconfigure::Server<grab_demo::params_colorConfig> server;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> colorSyncPolicy;

    message_filters::Synchronizer<colorSyncPolicy> *sync_;

    void rgb_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg);
    void dynamic_reconfigure_callback(grab_demo::params_colorConfig &config, uint32_t level);
    void imageCallback(const sensor_msgs::Image::ConstPtr &rgb_msg, const sensor_msgs::Image::ConstPtr &depth_msg);

public:
    Color_detect(/* args */);
    ~Color_detect();
};

Color_detect::Color_detect()
{
    visual_sub = nh.subscribe("camera/color/camera_info", 100, &Color_detect::rgb_info_callback, this);

    rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/color/image_raw", 1);
    depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/depth/image_raw", 1);
    sync_ = new message_filters::Synchronizer<colorSyncPolicy>(colorSyncPolicy(10), *rgb_sub, *depth_sub);

    sync_->registerCallback(std::bind(&Color_detect::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

    image_pub = nh.advertise<sensor_msgs::Image>("result", 1);

    server.setCallback(boost::bind(&Color_detect::dynamic_reconfigure_callback, this, _1, _2));
}

Color_detect::~Color_detect()
{
}

void Color_detect::imageCallback(const sensor_msgs::Image::ConstPtr &rgb_msg, const sensor_msgs::Image::ConstPtr &depth_msg)
{
    if (camera_info)
    {
        cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8);
        cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depimage = cv_depth_ptr->image;

        // 将图像从BGR颜色空间转换到HSV颜色空间
        cv::Mat hsv_image_raw;
        cv::cvtColor(cv_rgb_ptr->image, hsv_image_raw, cv::COLOR_RGB2HSV);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat hsv_image_erode;
        cv::Mat hsv_image_dilate;

        // 对HSV图像应用颜色阈值
        cv::Mat threshold_image;
        cv::inRange(hsv_image_raw,
                    cv::Scalar(color_hsv[color_select][0], color_hsv[color_select][1], color_hsv[color_select][2]),
                    cv::Scalar(color_hsv[color_select][3], color_hsv[color_select][4], color_hsv[color_select][5]),
                    threshold_image);

        cv::erode(threshold_image, hsv_image_erode, kernel);
        cv::dilate(hsv_image_erode, hsv_image_dilate, kernel);

        result_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, hsv_image_dilate).toImageMsg();
        image_pub.publish(result_img);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(hsv_image_dilate, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2)
                  { return cv::contourArea(c1) > cv::contourArea(c2); });

        if (!contours.empty())
        {
            std::vector<cv::Point> contour = contours[0];
            cv::Moments moments = cv::moments(contour);
            cv::Point newpos(moments.m10 / moments.m00, moments.m01 / moments.m00);

            dis = (depimage.at<ushort>(newpos.y, newpos.x)) / 1000.0;

            x = (newpos.x - camera_matrix.at<double>(0, 2)) / camera_matrix.at<double>(0, 0) * dis;
            y = (newpos.y - camera_matrix.at<double>(1, 2)) / camera_matrix.at<double>(1, 1) * dis;

            obg_msg.transform.translation.x = x;
            obg_msg.transform.translation.y = y;
            obg_msg.transform.translation.z = dis;
            obg_msg.transform.rotation.x = 0;
            obg_msg.transform.rotation.y = 0;
            obg_msg.transform.rotation.z = 0;
            obg_msg.transform.rotation.w = 1;

            obg_msg.header.stamp = ros::Time::now();
            obg_msg.header.frame_id = "camera_color_optical_frame";
            obg_msg.child_frame_id = "color_pose_link";
            tf_pub.sendTransform(obg_msg);
        }
    }
    else
    {

        ROS_ERROR("invalid camera info!");
    }
}

void Color_detect::rgb_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{

    bool K_valid = 0;
    bool D_valid = 0;
    if (!camera_info)
    {

        for (uint8_t i = 0; i < msg->K.size(); i++)
        {
            if (msg->K.at(i) != 0)
            {
                K_valid = 1;
                break;
            }
        }
        for (uint8_t i = 0; i < msg->D.size(); i++)
        {
            if (msg->D.at(i) != 0)
            {
                D_valid = 1;
                break;
            }
        }
        if (K_valid && D_valid)
        {
            camera_matrix = cv::Mat::zeros(3, 3, CV_64F);

            camera_dis = cv::Mat::zeros(1, 5, CV_64F);

            for (uint8_t i = 0; i < 3; i++)
            {
                for (uint8_t j = 0; j < 3; j++)
                {
                    camera_matrix.at<double>(i, j) = msg->K[i * 3 + j];
                }
            }
            for (uint8_t i = 0; i < 5; i++)
            {
                camera_dis.at<double>(0, i) = msg->D[i];
            }
            camera_info = 1;
        }
    }
}

void Color_detect::dynamic_reconfigure_callback(grab_demo::params_colorConfig &config, uint32_t level)
{
    // 使用ROS的rqt工具进行动态调阈值
    color_hsv[0][0] = config.HSV_H_MIN;
    color_hsv[0][1] = config.HSV_S_MIN;
    color_hsv[0][2] = config.HSV_V_MIN;
    color_hsv[0][3] = config.HSV_H_MAX;
    color_hsv[0][4] = config.HSV_S_MAX;
    color_hsv[0][5] = config.HSV_V_MAX;
    color_select = config.color;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_detect");
    Color_detect color_detect;
    ros::spin();
    return 0;
}
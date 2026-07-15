#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat img;
        img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        // 在这里对图像进行处理
        cv::imshow("Image", img);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_subscriber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
    cv::namedWindow("Image");
    ros::spin();
    cv::destroyWindow("Image");
    return 0;
}

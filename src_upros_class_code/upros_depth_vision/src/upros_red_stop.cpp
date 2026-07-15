#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>

int direction = 0; // 0-stop,1-go

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat img;
        cv::Mat hsv;
        // 图像消息转cv
        img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

        // 获取图像的长宽
        int width = img.cols;
        int height = img.rows;

        // BGR空间转HSV空间
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // 设置阈值
        cv::Scalar lower_red = cv::Scalar(0, 30, 30);
        cv::Scalar upper_red = cv::Scalar(10, 255, 255);
        cv::Scalar lower_red2 = cv::Scalar(170, 30, 30);
        cv::Scalar upper_red2 = cv::Scalar(180, 255, 255);

        // 掩码图像判断颜色阈值
        cv::Mat mask1;
        cv::inRange(hsv, lower_red, upper_red, mask1);
        cv::Mat mask2;
        cv::inRange(hsv, lower_red2, upper_red2, mask2);
        cv::Mat mask;
        cv::bitwise_or(mask1, mask2, mask);

        // 判断红色像素数量
        int count = cv::countNonZero(mask);

        // 红色像素数量决定是否停止
        std::cout << "count: " << count << std::endl;
        if (count > 30000)
        {
            direction = 0;
        }
        else
        {
            direction = 1;
        }
        cv::Mat red_image;
        cv::bitwise_and(img, img, red_image, mask);

        // 显示图像
        cv::imshow("Image", red_image);
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
    ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    cv::namedWindow("Image");
    direction = 1;
    while (ros::ok())
    {

        if (direction == 0) // stop
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            cmd_pub_.publish(cmd_vel);
        }
        else if (direction == 1) // go
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.1;
            cmd_pub_.publish(cmd_vel);
        }
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    cv::destroyWindow("Image");
    return 0;
}

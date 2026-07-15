#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>

static const std::string INPUT = "Input";
static const std::string OUTPUT = "Output";

using namespace std;
using namespace cv;
int static times;

Mat srcImg;   // 原始图片
Mat roi;      // 框选后图片
Mat img;      // 处理图片
Mat output;   // 结果输出图片
Rect roiRect; // 感兴趣区域
Point pt;     // 初始坐标
RNG rng;
bool draw = false;

void OnMouseAction(int event, int x, int y, int flags, void *ustc)
{
    img = srcImg.clone();

    switch (event)
    {
    case EVENT_LBUTTONDOWN:
    {
        pt.x = x;
        pt.y = y;
        draw = true;
        std::cout << "at(" << x << "," << y << ")value is:" << static_cast<int>(img.at<uchar>(cv::Point(x, y))) << std::endl;
        break;
    }
    case EVENT_MOUSEMOVE:
    {
        if (draw == true)
        {
            roiRect.x = min(x, pt.x);
            roiRect.y = min(y, pt.y);
            roiRect.width = abs(x - pt.x);
            roiRect.height = abs(y - pt.y);
            rectangle(img, roiRect, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2, 8);
            break;
        }
        break;
    }
    case EVENT_LBUTTONUP:
    {
        if (roiRect.width > 0 && roiRect.height > 0 && draw == true)
        {
            roi = Scalar::all(0);
            roi = img(Rect(roiRect.x, roiRect.y, roiRect.width, roiRect.height));
            rectangle(img, roiRect, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 2, 8);

            for (int i = roiRect.x; i < roiRect.x + roiRect.width; i++)
            {

                for (int j = roiRect.y; j < roiRect.y + roiRect.height; j++)
                {
                    if (output.at<uchar>(cv::Point(i, j)) != 255)
                    {
                        output.at<uchar>(cv::Point(i, j)) = 255;
                    }
                }
            }
            cv::imshow(OUTPUT, output);
        }
        draw = false;
        break;
    }
    default:
        break;
    }
    imshow(INPUT, img);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGB_GRAY");
    ros::NodeHandle nh;

    const std::string path = ros::package::getPath("upros_slam");

    const std::string map_file = path + "/maps/my_lab.pgm";
    const std::string save_path = path + "/maps/my_lab_save.pgm";

    cv::namedWindow(INPUT);
    cv::namedWindow(OUTPUT);

    cv::Mat map_img = cv::imread(map_file, 0);

    if (map_img.empty())
    {
        ROS_ERROR("Cant load pgm");
    }

    map_img.convertTo(map_img, CV_8U);
    int width = map_img.cols;
    int height = map_img.rows;
    srcImg = map_img.clone();
    output = srcImg.clone();
    setMouseCallback(INPUT, OnMouseAction, 0);
    cv::imshow(OUTPUT, output);
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        /* code */
        cv::imshow(INPUT, srcImg);
        if (cv::waitKey(0) == 's')
        {
            cv::imwrite(save_path, output); // 保存处理后的图片
        }
        if (cv::waitKey(0) == 27)
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
#ifndef UPROS_ARM_H
#define UPROS_ARM_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <upros_message/SingleServo.h>
#include <upros_message/MultipleServo.h>

class UPROS_ARM
{

private:
    ros::Publisher single_joint_pub_;
    ros::Publisher multiple_joint_pub_;

    // 舵盘的偏移量
    std::vector<int> servo_bias_ = {0, 0, 0, 0, 0, 0};

    // 1,2,3,4四个逆运算末端关节的舵机位置
    std::vector<int> servo_angle = {0, 0, 0, 0};

    // 各个关节长度，单位毫米
    int l0 = 110;
    int l1 = 61;
    int l2 = 81;
    int l3 = 200;

    int current_claw_angle = 0;
    int current_wrist_angle = 0;

public:
    UPROS_ARM()
    {
        ros::NodeHandle n("~");
        n.getParam("servo_bias_1", servo_bias_[0]);
        n.getParam("servo_bias_2", servo_bias_[1]);
        n.getParam("servo_bias_3", servo_bias_[2]);
        n.getParam("servo_bias_4", servo_bias_[3]);
        n.getParam("servo_bias_5", servo_bias_[4]);
        n.getParam("servo_bias_6", servo_bias_[5]);

        single_joint_pub_ = n.advertise<upros_message::SingleServo>("/single_servo_topic", 10);
        multiple_joint_pub_ = n.advertise<upros_message::MultipleServo>("/multiple_servo_topic", 10);

        ROS_INFO("The value of all bias is %d, %d, %d, %d, %d, %d", servo_bias_[0], servo_bias_[1], servo_bias_[2], servo_bias_[3], servo_bias_[4], servo_bias_[5]);
    }

    ~UPROS_ARM() {}

    void claw_open()
    {
        upros_message::SingleServo single_servo;
        single_servo.ID = 6;
        single_servo.Rotation_Speed = 50;
        single_servo.Target_position_Angle = 580;
        single_joint_pub_.publish(single_servo);
    }

    void claw_close()
    {
        upros_message::SingleServo single_servo;
        single_servo.ID = 6;
        single_servo.Rotation_Speed = 50;
        single_servo.Target_position_Angle = -300;
        single_joint_pub_.publish(single_servo);
    }

    //回到零位，夹爪闭合
    void go_home()
    {
        upros_message::MultipleServo multiple_servo;
        for (int i = 0; i < 6; i++)
        {
            upros_message::SingleServo single_servo;
            single_servo.ID = i + 1;
            single_servo.Rotation_Speed = 50;
            single_servo.Target_position_Angle = 0 + servo_bias_[i];
            if(i == 4){
                single_servo.Target_position_Angle = 0;
            }
            if(i == 5){
                single_servo.Target_position_Angle = -400;
            }
            multiple_servo.servo_gather.push_back(single_servo);
        }
        multiple_joint_pub_.publish(multiple_servo);
    }

    // 逆运动学分析，输入为距离机械臂基座的X，Y，Z坐标，抓取的角度，输出计算结果
    int inverseAnalysis(int dis_x, int dis_y, int dis_z, int alpha)
    {
        double theta1;
        // 计算舵机1（云台转角）
        if (dis_x == 0 && dis_y != 0)
        {
            theta1 = 0;
        }
        else if (dis_x > 0 && dis_y == 0)
        {
            theta1 = 90;
        }
        else if (dis_x < 0 && dis_y == 0)
        {
            theta1 = -90;
        }
        else
        {
            theta1 = std::atan(float(dis_x) / float(dis_y)) * 180.0 / M_PI;
        }

        // 计算在y_z平面上，整体的距离
        double y123 = std::sqrt(dis_y * dis_y + dis_x * dis_x);

        // 假设舵机3关节不变，计算y12和z12
        double y12 = y123 - l3 * std::cos(alpha * M_PI / 180.0);
        double z12 = dis_z - l0 - l3 * std::sin(alpha * M_PI / 180);

        if (z12 < -l0)
        {
            return 1; // 低于桌面了，肯定错误
        }
        if (std::sqrt(y12 * y12 + z12 * z12) > (l1 + l2))
        {
            return 2; // y12的长度比两个连杆和还长，肯定错误
        }

        double ccc = std::acos(y12 / std::sqrt(y12 * y12 + z12 * z12));

        // 余弦定理
        double bb = (y12 * y12 + z12 * z12 + l1 * l1 - l2 * l2) / (2 * l1 * std::sqrt(y12 * y12 + z12 * z12));
        if (bb > 1 || bb < -1)
        {
            return 3; // 超过cos值，肯定错误
        }
        double bbb = std::acos(bb);

        double aa = (y12 * y12 + z12 * z12 + l2 * l2 - l1 * l1) / (2 * l2 * std::sqrt(y12 * y12 + z12 * z12));
        if (aa > 1 || aa < -1)
        {
            return 4; // 超过cos值，肯定错误
        }
        double aaa = std::acos(aa);

        int zf_flag;
        if (z12 < 0)
        {
            zf_flag = -1;
        }
        else
        {
            zf_flag = 1;
        }

        // 计算第二个关节
        double theta2 = M_PI / 2 - (ccc * zf_flag + bbb);
        theta2 = theta2 * 180.0 / M_PI;
        if (theta2 > 135.0 || theta2 < -135.0)
        {
            return 5;
        }

        // 计算第三个关节
        double theta3 = aaa + bbb;
        theta3 = theta3 * 180.0 / M_PI;
        if (theta3 > 135.0 || theta3 < -135.0)
        {
            return 6;
        }

        // 计算第四个关节
        double theta4 = M_PI / 2 - alpha * M_PI / 180.0 - theta2 * M_PI / 180.0 - theta3 * M_PI / 180.0;
        theta4 = theta4 * 180.0 / M_PI;
        if (theta4 > 135.0 || theta4 < -135.0)
        {
            return 7;
        }

        servo_angle[0] = -theta1 * 10;
        servo_angle[1] = -theta2 * 10;
        servo_angle[2] = -theta3 * 10;
        servo_angle[3] = -theta4 * 10;

        return 0;
    }

    int inverseFind(double x, double y, double z)
    {
        int best_alpha = 0;
        std::vector<int> alpha_list;
        int flag = 0;

        // 遍历-45度到0度角，解算出可行的逆运算都记录下来
        for (int i = -45; i <= 0; ++i)
        {
            if (inverseAnalysis(x, y, z, i) == 0)
            {
                alpha_list.push_back(i);
            }
        }

        // 使用与水平面夹角最大的角度进行抓取，更加方便
        if (alpha_list.size() > 0)
        {
            best_alpha = *std::min_element(alpha_list.begin(), alpha_list.end());
            flag = 1;
        }

        // 根据夹角与xyz坐标值，求解出四个关节舵机的角度
        if (flag)
        {
            inverseAnalysis(x, y, z, best_alpha);
            std::cout << "Alpha: " << best_alpha << std::endl;
            std::cout << "angle 0: " << servo_angle[0]
                      << " angle 1: " << servo_angle[1]
                      << " angle 2: " << servo_angle[2]
                      << " angle 3: " << servo_angle[3]
                      << std::endl;
        }

        return flag;
    }

    //基于逆运算运动去抓取，需要夹爪保持张开
    void inverseMoveToGrab()
    {
        upros_message::MultipleServo multiple_servo;
        for (int i = 0; i < 6; i++)
        {
            upros_message::SingleServo single_servo;
            single_servo.ID = i + 1;
            single_servo.Rotation_Speed = 50;
            single_servo.Target_position_Angle = servo_angle[i] + servo_bias_[i];
            if(i == 4){
                single_servo.Target_position_Angle = 0;
            }
            if(i == 5){
                single_servo.Target_position_Angle = 580;
            }
            multiple_servo.servo_gather.push_back(single_servo);
        }
        multiple_joint_pub_.publish(multiple_servo);
    }

    //基于逆运算运动去放置，需要夹爪保持闭合
    void inverseMoveToPut()
    {
        upros_message::MultipleServo multiple_servo;
        for (int i = 0; i < 6; i++)
        {
            upros_message::SingleServo single_servo;
            single_servo.ID = i + 1;
            single_servo.Rotation_Speed = 50;
            single_servo.Target_position_Angle = servo_angle[i] + servo_bias_[i];
            if(i == 4){
                single_servo.Target_position_Angle = 0;
            }
            if(i == 5){
                single_servo.Target_position_Angle = -400;
            }
            multiple_servo.servo_gather.push_back(single_servo);
        }
        multiple_joint_pub_.publish(multiple_servo);
    }   
};

#endif
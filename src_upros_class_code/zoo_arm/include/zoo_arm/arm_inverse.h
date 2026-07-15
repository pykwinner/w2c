#ifndef ARM_INVERSE_H
#define ARM_INVERSE_H

#include <cmath>
#include <vector>
#include "ros/ros.h"

using namespace std;

class ArmInverse
{

public:
    ArmInverse() {}
    ~ArmInverse() {}

private:
    // 机械臂joint0到基座的高度
    float base_height = 125;
    // 机械臂大臂长度
    float l1 = 135;
    // 机械臂小臂长度
    float l2 = 160;
    // 电机每一步转动对应的弧度
    float step_angle = 0.01125 * M_PI / 180.0;
    // 吸盘和机械臂joint2 在x,y平面的偏移量
    float xy_offset = 40;
    // 吸盘和机械臂joint2 在z 高度上的偏移量
    float z_offset = 50;
    // 机械臂初始化角度
    float initial_angles[3] = {0, M_PI / 2, M_PI * 1 / 8};

    int STEP_RANGE[3][2] = {{-8000, 8000}, {0, 6000}, {-8000, 500}};

public:
    // 对输入的吸盘相对于机械臂joint1位置转化为 机械臂末端相对于机械臂joint1位置，并进行偏移调整
    vector<int> adjust_position(int x, int y, int z);

    // 对计算出的theta1, theta2, theta3 进行限位
    vector<int> step_limiter(vector<int> thetas);

    // 如果每个关节的步进数都在工作范围之内，则返回True;否则返回False
    bool check_steps_reachable(vector<int> steps);

    bool check_valid(int relative_x_dist, int relative_y_dist, int relative_z_dist);

    vector<float> inverse_kinematics(int x, int y, int z);

    vector<int> calculate_steps(vector<float> target_angles);

    vector<int> get_steps_from_absolute_pos(int x, int y, int z);
};

#endif

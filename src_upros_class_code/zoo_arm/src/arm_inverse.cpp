#include "arm_inverse.h"

// 对输入的吸盘相对于机械臂joint1位置转化为 机械臂末端相对于机械臂joint1位置，并进行偏移调整
vector<int> ArmInverse::adjust_position(int x, int y, int z)
{
    float angle = atan2(x, y);
    int x_adjusted = x - xy_offset * sin(angle);
    int y_adjusted = y - xy_offset * cos(angle);
    int z_adjusted = z + z_offset;
    // 机械臂末端相对于机械臂基座位置,并进行偏移调整后的坐标(单位:cm)
    vector<int> adjust_poses = {x_adjusted, y_adjusted, z_adjusted};
    return adjust_poses;
}


// 对计算出的theta1, theta2, theta3 进行限位
vector<int> ArmInverse::step_limiter(vector<int> thetas)
{
    for (int i = 0; i < 3; i++)
    {
        if (thetas[i] >= STEP_RANGE[i][1])
        {
            thetas[i] = STEP_RANGE[i][1];
        }
        else if (thetas[i] <= STEP_RANGE[i][0])
        {
            thetas[i] = STEP_RANGE[i][0];
        }
    }
    return thetas;
}

// 如果每个关节的步进数都在工作范围之内，则返回True;否则返回False
bool ArmInverse::check_steps_reachable(vector<int> steps)
{
    if (steps[0] > STEP_RANGE[0][1] || steps[0] < STEP_RANGE[0][0])
    {
        return false;
    }

    if (steps[1] > 6000)
    {
        return false;
    }

    if (steps[2] < -10000 || steps[2] > 500)
    {
        return false;
    }

    if (steps[2] > -1700)
    {
        if (steps[1] < steps[2])
        {
            return false;
        }
    }

    if (steps[2] < -1700 && steps[2] >= -10000)
    {
        if (steps[1] < -1700 || (steps[1] - steps[2]) > 11700)
        {
            return false;
        }
    }

    return true;
}

// 校验机械臂限位
bool ArmInverse::check_valid(int relative_x_dist, int relative_y_dist, int relative_z_dist)
{
    float r = sqrt(pow(relative_x_dist, 2) + pow(relative_y_dist, 2) + pow(relative_z_dist, 2));
    if (r < 105 || r >= (l1 + l2))
    {
        return false;
    }
    return true;
}

vector<float> ArmInverse::inverse_kinematics(int x, int y, int z)
{
    // 底部关节旋转角度 theta1
    float theta1 = atan2(x, y);

    // 目标点在机械臂平面上的投影距离
    float r = sqrt(pow(x, 2) + pow(y, 2));

    // joint 和目标点 连线长度 d
    float d = sqrt(pow(r, 2) + pow(z, 2));

    vector<float> zeros = {0, 0, 0};

    if (d > l1 + l2 || d < abs(l1 - l2))
    {
        return zeros;
    }

    float alpha = acos((pow(l1, 2) + pow(d, 2) - pow(l2, 2)) / (2 * l1 * d));

    float beta = acos((pow(l1, 2) + pow(l2, 2) - pow(d, 2)) / (2 * l1 * l2));

    float theta = atan2(z, r);

    float theta2 = theta + alpha;

    float theta3 = beta - (M_PI / 2 - theta2);

    vector<float> thetas = {theta1, theta2, theta3};

    return thetas;
}

// 计算旋转角度
vector<int> ArmInverse::calculate_steps(vector<float> target_angles)
{
    float theta1 = target_angles[0];
    float theta2 = target_angles[1];
    float theta3 = target_angles[2];

    float theta1_0 = initial_angles[0];
    float theta2_0 = initial_angles[1];
    float theta3_0 = initial_angles[2];

    float delta_theta1 = theta1 - theta1_0;
    float delta_theta2 = theta2 - theta2_0;
    float delta_theta3 = theta3 - theta3_0;

    int step1 = delta_theta1 / step_angle;
    int step2 = delta_theta2 / step_angle;
    int step3 = delta_theta3 / step_angle;

    vector<int> steps = {-step1, step3, step2};
    return steps;
}

// 根据xyz坐标计算机械臂电机运动步数
vector<int> ArmInverse::get_steps_from_absolute_pos(int x, int y, int z)
{
    vector<int> zero = {0, 0, 0};

    // #末端执行器到机械臂joint1在x,y,z上的变化
    float dx = x;
    float dy = y;
    float dz = z - base_height;

    // 根据末端执行器对joint1的位置变换， 计算 小臂末端l2_end对joint1的位置变换
    vector<int> joint_end = adjust_position(dx, dy, dz);

    ROS_INFO("joint_end = %.2f, %.2f,%.2f", joint_end[0], joint_end[1], joint_end[2]);

    bool is_work_space = check_valid(joint_end[0], joint_end[1], joint_end[2]);

    if (!is_work_space)
    {
        ROS_INFO("not is_work_space!!!!");
        return zero;
    }

    vector<float> target_angles = inverse_kinematics(joint_end[0], joint_end[1], joint_end[2]);
    ROS_INFO("target_angles = %.2f, %.2f,%.2f", target_angles[0], target_angles[1], target_angles[2]);

    vector<int> steps = calculate_steps(target_angles);
    ROS_INFO("current steps = %d, %d,%d", steps[0], steps[1], steps[2]);

    bool check_step = check_steps_reachable(steps);
    
    if (!check_step)
    {
        ROS_INFO("check_step error!!!!");
        return zero;
    }
    return steps;
}

#ifndef GO_STRIGHT_PLANNER_H
#define GO_STRIGHT_PLANNER_H

#include <ros/ros.h>
#include "base_planner.h"
#include <fstream>
#include <iostream>
#include "real_pid.h"
#include <sensor_msgs/Range.h>

/*
前进的规划器，接收一个目标点，通过tf变换获取某个坐标系下当前位姿，先自旋到朝向目标点，然后直线前进，前方如果出现障碍物则停止
*/
namespace core_move
{
    class GoStraightPlanner : public BasePlanner
    {
    public:
        GoStraightPlanner(boost::shared_ptr<tf2_ros::Buffer> tf,
                          geometry_msgs::PoseStamped target,
                          std::string parent_frame,
                          std::string child_frame);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

    private:
        GoStraightPlanner();

        boost::shared_ptr<tf2_ros::Buffer> tf_;

        enum State
        {
            INIT = 0,
            TURNAROUND,
            GOSTRIGHT
        };

        State state_ = INIT;
        geometry_msgs::PoseStamped target_;
        geometry_msgs::PoseStamped robot_pose_;

        std::float_t error_dist;
        std::float_t error_angle;

        std::string parent_frame_, child_frame_;

        RealPID turn_pid_;
        RealPID straight_pid_;
        RealPID straight_keep_pid_;

        double calAngleError();

        double calDistError();

        double angle_err;
        double dist_err;

        bool getRobotPose(boost::shared_ptr<tf2_ros::Buffer> tf, geometry_msgs::PoseStamped &robot_pose);

    };
};

#endif // GO_STRIGHT_PLANNER_H

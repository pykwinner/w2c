#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H

#include "common.h"

namespace core_move
{
    class BasePlanner
    {
    public:
        BasePlanner();

        virtual bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) = 0;

        virtual bool isGoalReached() = 0;

        virtual ~BasePlanner() {}

    protected:
        virtual bool getRobotPose(boost::shared_ptr<tf2_ros::Buffer> tf, geometry_msgs::PoseStamped &robot_pose);

        std::string base_frame_, global_frame_;
    };
}

#endif // BASE_PLANNER_H

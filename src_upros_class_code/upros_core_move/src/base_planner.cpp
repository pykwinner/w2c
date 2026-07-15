#include "upros_core_move/base_planner.h"

namespace core_move
{
    BasePlanner::BasePlanner()
    {
        // core_move_node namespace.
        ros::NodeHandle private_nh("~");
        private_nh.param("global_frame", global_frame_, std::string("map"));
        private_nh.param("robot_base_frame", base_frame_, std::string("base_link"));
    }

    bool BasePlanner::getRobotPose(boost::shared_ptr<tf2_ros::Buffer> tf, geometry_msgs::PoseStamped &robot_pose)
    {
        ros::Time now = ros::Time::now();
        ros::Rate r(2);

        // test tf work correctly
        while (ros::ok())
        {
            if (tf->canTransform(global_frame_, base_frame_, ros::Time(0), ros::Duration(0.2)))
            {
                break;
            }
            else
            {
                ROS_ERROR("no tf, try again!!");
                r.sleep();
            }
        }

        if (tf->canTransform(global_frame_, base_frame_, ros::Time(0), ros::Duration(0.2)))
        {
            geometry_msgs::PoseStamped original_pose;
            original_pose.header.stamp = ros::Time(0);
            original_pose.pose.orientation.w = 1.0;
            original_pose.header.frame_id = base_frame_;
            tf->transform(original_pose, robot_pose, global_frame_);
            ROS_INFO("");
        }
        else
        {
            ROS_ERROR("No Transform available Error looking up robot pose");
            ROS_ERROR(" failed!");
            return false;
        }

        return true;
    }
}

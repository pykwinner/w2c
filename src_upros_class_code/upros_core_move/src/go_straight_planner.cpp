#include "upros_core_move/go_straight_planner.h"
#include "upros_core_move/real_pid.h"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

namespace core_move
{
    GoStraightPlanner::GoStraightPlanner(boost::shared_ptr<tf2_ros::Buffer> tf,
                                         geometry_msgs::PoseStamped target,
                                         std::string parent_frame,
                                         std::string child_frame) : state_(TURNAROUND),
                                                                    target_(target),
                                                                    tf_(tf),
                                                                    parent_frame_(parent_frame),
                                                                    child_frame_(child_frame)
    {
        ros::NodeHandle global_nh;
        global_nh.param("/core_move_node/error_dist", error_dist, std::float_t(0.25));
        global_nh.param("/core_move_node/error_angle", error_angle, std::float_t(0.25));

        getRobotPose(tf_, robot_pose_);

        angle_err = calAngleError();
        dist_err = calDistError();

        error_dist = 0.10;
        error_angle = 0.10;

        turn_pid_ = RealPID(4.0, 0.0, 0, 0.5, -0.5, 1.0, 10);
        straight_pid_ = RealPID(0.8, 0.05, 0, 0.3, 0, 10, 10);
        straight_keep_pid_ = RealPID(0.8, 0.05, 0, 0.3, 0, 10, 10);
    }

    // 使用基站tf作为参考，获取base_link到station_frame的坐标变换
    bool
    GoStraightPlanner::getRobotPose(boost::shared_ptr<tf2_ros::Buffer> tf, geometry_msgs::PoseStamped &robot_pose)
    {
        ros::Time now = ros::Time::now();
        // 能够获取到parent_link到child_link的坐标变换
        if (tf->canTransform(parent_frame_, child_frame_, now, ros::Duration(0.2)))
        {
            geometry_msgs::PoseStamped original_pose;
            original_pose.header.stamp = now;
            original_pose.pose.orientation.w = 1.0;
            original_pose.header.frame_id = child_frame_;
            tf->transform(original_pose, robot_pose, parent_frame_);
        }
        else
        {
            // 不能获取坐标变换
            ROS_ERROR("Cant get position!!!");
            return false;
        }
        return true;
    }

    // 速度计算
    bool GoStraightPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {

        // 如果获取不到当前信息，自旋找角度
        if (!getRobotPose(tf_, robot_pose_))
        {
            return false;
        }

        // 计算角度偏量
        angle_err = calAngleError();

        // 计算位置偏量
        dist_err = calDistError();

        if (state_ == TURNAROUND)
        {
            double angle_err = calAngleError();
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = turn_pid_.calOutput(angle_err);
            if (fabs(angle_err) < error_angle)
            {
                state_ = GOSTRIGHT;
                turn_pid_.clearIntegrator();
                straight_keep_pid_.clearIntegrator();
                straight_keep_pid_.clearIntegrator();
                ros::Duration(0.1).sleep();
            }
        }
        else // state_ == GOSTRIGHT
        {

            cmd_vel.linear.x = straight_pid_.calOutput(dist_err);
            cmd_vel.angular.z = straight_keep_pid_.calOutput(angle_err);
          }
        return true;
    }

    bool GoStraightPlanner::isGoalReached()
    {
        if (fabs(calDistError()) < error_dist)
        {
            turn_pid_.clearIntegrator();
            straight_keep_pid_.clearIntegrator();
            straight_keep_pid_.clearIntegrator();
            ROS_WARN("reach goal.");
            return true;
        }
        return false;
    }

    double GoStraightPlanner::calAngleError()
    {
        double robot_yaw = tf2::getYaw(robot_pose_.pose.orientation);
        double dx = target_.pose.position.x - robot_pose_.pose.position.x;
        double dy = target_.pose.position.y - robot_pose_.pose.position.y;
        double target_yaw = atan2(dy, dx);
        double angle_error = target_yaw - robot_yaw;
        if (angle_error < -1 * 3.14)
            angle_error = 2 * 3.14 + angle_error;
        if (angle_error > 3.14)
            angle_error = angle_error - 2 * 3.14;
        return angle_error;
    }

    double GoStraightPlanner::calDistError()
    {
        double dx = target_.pose.position.x - robot_pose_.pose.position.x;
        double dy = target_.pose.position.y - robot_pose_.pose.position.y;
        double dist_error = sqrt(dx * dx + dy * dy);
        return dist_error;
    }
}

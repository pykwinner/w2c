#ifndef UPROS_DEMO_H
#define UPROS_DEMO_H

#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

bool success;

class UPROS : moveit::planning_interface::MoveGroupInterface
{
private:
    ros::NodeHandle n;
    std::string group_name;
    std::string end_effector_link;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

public:
    geometry_msgs::Pose current_pose;

    UPROS(std::string move_group_name) : moveit::planning_interface::MoveGroupInterface(move_group_name)
    {
        // this->setGoalPositionTolerance(0.005);
        // this->setGoalOrientationTolerance(0.005);
        // this->setGoalJointTolerance(0.005);
        // this->allowReplanning(true);
        // this->setPlanningTime(10.0);
        // this->setPlannerId("TRRT");
        this->setMaxVelocityScalingFactor(1.0);
        this->setMaxAccelerationScalingFactor(1.0);

        // 获取规划组参考坐标系
        std::string planning_frame = this->getPlanningFrame();
        ROS_INFO("============ Reference frame: %s", planning_frame.c_str());

        // 获取末端执行器的link
        end_effector_link = this->getEndEffectorLink();
        ROS_INFO("============ End effector: %s", end_effector_link.c_str());
    }

    geometry_msgs::Pose get_current_pose()
    {
        double roll, pitch, yaw;
        geometry_msgs::Quaternion q1;
        current_pose = this->getCurrentPose(end_effector_link).pose;
        q1.x = current_pose.orientation.x;
        q1.y = current_pose.orientation.y;
        q1.z = current_pose.orientation.z;
        q1.w = current_pose.orientation.w;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(q1, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return current_pose;
    }

    void move_Home()
    {
        this->setNamedTarget("home");
        this->move();
    }

    void move_Grab()
    {
        this->setNamedTarget("grab");
        this->move();
    }

    void move_Put()
    {
        this->setNamedTarget("put");
        this->move();
    }

    bool go_to_joint(const std::vector<double> &joint_values)
    {
        this->setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (this->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            this->execute(my_plan);
            return true;
        }
        return false;
    }

    bool go_to_pose(double x, double y, double z)
    {
        geometry_msgs::Pose target_pose;
        target_pose = this->getCurrentPose(end_effector_link).pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        this->setStartStateToCurrentState();
        this->setPoseTarget(target_pose);
        moveit::core::MoveItErrorCode success = this->plan(arm_plan);
        if (success)
        {
            this->execute(arm_plan);
            return true;
        }
        return false;
    }

    bool go_to_pose(double x, double y, double z, double rx, double ry, double rz)
    {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(rx / 57.29578, ry / 57.29578, rz / 57.29578);
        target_pose.orientation.x = myQuaternion.getX();
        target_pose.orientation.y = myQuaternion.getY();
        target_pose.orientation.z = myQuaternion.getZ();
        target_pose.orientation.w = myQuaternion.getW();
        this->setStartStateToCurrentState();
        this->setPoseTarget(target_pose);
        moveit::core::MoveItErrorCode success = this->plan(arm_plan);
        if (success)
        {
            this->execute(arm_plan);
            return true;
        }
        return false;
    }
};

#endif

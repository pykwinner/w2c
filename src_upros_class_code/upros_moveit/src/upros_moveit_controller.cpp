#include <ros/ros.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <upros_message/SingleServo.h>
#include <upros_message/MultipleServo.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <vector>

using namespace std;

class ArmJointTrajectory
{
protected:
    ros::NodeHandle nh;
    std::string action_name_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

    control_msgs::FollowJointTrajectoryActionResult result_;
    control_msgs::FollowJointTrajectoryActionGoal goal_;

    upros_message::SingleServo single_servo_msgs;
    upros_message::MultipleServo multip_servo_msgs;

    ros::Publisher multip_servo_pub;

public:
    ArmJointTrajectory(std::string name) : as_(nh, name, false),
                                           action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&ArmJointTrajectory::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&ArmJointTrajectory::preemptCB, this));
        as_.start();
        ROS_INFO("-------action start!-------");
        multip_servo_msgs.servo_gather.resize(5);
        multip_servo_pub = nh.advertise<upros_message::MultipleServo>("multiple_servo_topic", 50);
    }
    ~ArmJointTrajectory(void)
    {
    }

    void goalCB()
    {
        ROS_INFO("-------goal is receive!-------");
        std::vector<trajectory_msgs::JointTrajectoryPoint> points_ =
            as_.acceptNewGoal()->trajectory.points;
        int pos_length = points_.size();
        ROS_INFO("pos length = %d", pos_length);
        for (int i = 0; i < 5; i++)
        {
            double pos = points_.at(pos_length - 1).positions[i];
            ROS_INFO("pos = %.2f", pos);
            single_servo_msgs.ID = i + 1;
            single_servo_msgs.Target_position_Angle = pos * 180 / M_PI * 10;
            single_servo_msgs.Rotation_Speed = 50;
            multip_servo_msgs.servo_gather[i] = single_servo_msgs;
        }
        multip_servo_pub.publish(multip_servo_msgs);
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = 0;
        as_.setSucceeded(result);
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "upros_arm_driver");
    ArmJointTrajectory armJointTrajectory("/arm_joint_controller/follow_joint_trajectory");
    ros::spin();
    return 0;
}
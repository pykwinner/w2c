#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>

using namespace std;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void Move2goal(MoveBaseClient& ac, double x, double y, double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 1 Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shoot_robot_base");
    ros::NodeHandle nh;

    ros::ServiceClient shoot_close_client;
    std_srvs::Empty empty_srv;

    shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    MoveBaseClient ac("move_base", true);

    ac.waitForServer();
    Move2goal(ac, 0.73, 0.25, -1.5707);
    shoot_close_client.call(empty_srv);
    Move2goal(ac, 1.76, -0.06, 0);
    shoot_close_client.call(empty_srv);
    //Move2goal(ac, 0.26, 2.13, 1.5707);
    //shoot_close_client.call(empty_srv);
    // Move2goal(ac, 1.6, 3.0, 0.0);
    //shoot_close_client.call(empty_srv);
    // Move2goal(ac, 3.0, 3.0, 1.5707);
    //shoot_close_client.call(empty_srv);
    // Move2goal(ac, 3.0, 1.6, 1.5707);
    //shoot_close_client.call(empty_srv);
    // Move2goal(ac, 3.0, 1.4, 1.5707);
    //shoot_close_client.call(empty_srv);
    // Move2goal(ac, 3.0, 0.0, 3.1415);
    //shoot_close_client.call(empty_srv);
    // Move2goal(ac, 1.6, 0.0, 3.1415);
    //shoot_close_client.call(empty_srv);

    move_base_msgs::MoveBaseGoal goal3;
    goal3.target_pose.pose.position.x = 0.0;
    goal3.target_pose.pose.position.y = 0.0;
    goal3.target_pose.pose.orientation.z = 0.0;
    goal3.target_pose.pose.orientation.w = 1.0;
    goal3.target_pose.header.frame_id = "map";
    goal3.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal3);
    ROS_INFO("Send Goal Home !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back !!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    return 0;

}

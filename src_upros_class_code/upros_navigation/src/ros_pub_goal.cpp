#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <sstream>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigationDemo
{

public:

    NavigationDemo() : move_base_("move_base", true)
    {
        while (!move_base_.waitForServer(ros::Duration(60)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    }

    void doneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        ROS_INFO("navigation done! status:%s result:%s", state.toString().c_str(), "");
    }

    void activeCallback()
    {
        ROS_INFO("[Navi] navigation has be actived");
    }

    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
    {
        ROS_INFO("[Navi] navigation feedback\r\n%s", "");
    }

    bool gotoPoint(double x, double y, double yaw)
    {
        ROS_INFO("[Navi] goto (%f, %f, %f)", x, y, yaw);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw * M_PI / 180.0);
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        move_base_.sendGoal(goal, boost::bind(&NavigationDemo::doneCallback, this, _1, _2),
                            boost::bind(&NavigationDemo::activeCallback, this),
                            boost::bind(&NavigationDemo::feedbackCallback, this, _1));

        bool finished_before_timeout = move_base_.waitForResult(ros::Duration(60));

        if (!finished_before_timeout)
        {
            move_base_.cancelGoal();
            ROS_INFO("Timed out achieving goal");
            return false;
        }
        else
        {
            actionlib::SimpleClientGoalState state = move_base_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("reach goal (%f, %f, %f) succeeded!", x, y, yaw);
                return true;
            }
            else
            {
                return false;
            }
        }
    }

private:
    MoveBaseClient move_base_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_demo");

    std::vector<double> goalListX = {1.0, 3.0};
    std::vector<double> goalListY = {0.0, 0.0};
    std::vector<double> goalListYaw = {0.0, 0.0};

    if (goalListX.size() != goalListY.size() || goalListX.size() != goalListYaw.size())
    {
        ROS_ERROR("Goal lists must have the same size");
        return -1;
    }

    std::vector<std::vector<double>> goals;
    for (size_t i = 0; i < goalListX.size(); ++i)
    {
        goals.push_back({goalListX[i], goalListY[i], goalListYaw[i]});
    }

    ros::Rate r(1);
    r.sleep();

    NavigationDemo navi;

    navi.gotoPoint(goals[0][0], goals[0][1], goals[0][2]);
    ros::Duration(3).sleep();

    navi.gotoPoint(goals[1][0], goals[1][1], goals[1][2]);
    ros::Duration(2).sleep();

    return 0;
}
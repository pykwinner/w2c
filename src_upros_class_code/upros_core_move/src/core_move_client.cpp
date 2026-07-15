#include "upros_core_move/core_move_client.h"

namespace core_move
{
    CoreMoveClient::CoreMoveClient(ros::NodeHandle &global_nh, ros::NodeHandle &private_nh) : m_global_nh(global_nh),
                                                                                              m_private_nh(private_nh)

    {
        m_click_point_sub_ = m_global_nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &CoreMoveClient::clickedPointCallback, this);
        client = std::shared_ptr<actionlib::SimpleActionClient<upros_message::CoreMoveAction>>(new actionlib::SimpleActionClient<upros_message::CoreMoveAction>("core_move", true));
        if (!client->waitForServer(ros::Duration(10)))
        {
            ROS_ERROR("Cant connect");
        }
        ros::spin();
    }

    void CoreMoveClient::doneCb(const actionlib::SimpleClientGoalState &state, const upros_message::CoreMoveResultConstPtr &result)
    {
        ROS_WARN("target done!!");
    }

    void CoreMoveClient::activeCb()
    {
    }

    void CoreMoveClient::feedbackCb(const upros_message::CoreMoveFeedbackConstPtr &feedback)
    {
    }

    void CoreMoveClient::clickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg)
    {
        upros_message::CoreMoveGoal goal;
        goal.cmd = 1;
        goal.target_pose.pose.position.x = msg->point.x;
        goal.target_pose.pose.position.y = msg->point.y;
        client->sendGoal(goal, boost::bind(&CoreMoveClient::doneCb, this, _1, _2),
                         boost::bind(&CoreMoveClient::activeCb, this),
                         boost::bind(&CoreMoveClient::feedbackCb, this, _1));
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core_move_client");
    ros::NodeHandle global_nh;
    ros::NodeHandle private_nh("~");
    core_move::CoreMoveClient cmc(global_nh, private_nh);
    return 0;
}
#ifndef CORE_MOV_CLIENT_H

#define CORE_MOV_CLIENT_H

#include "common.h"
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <upros_message/CoreMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace core_move
{
    class CoreMoveClient
    {
    public:
        CoreMoveClient(ros::NodeHandle &global_nh, ros::NodeHandle &private_nh);

    private:
        std::shared_ptr<actionlib::SimpleActionClient<upros_message::CoreMoveAction>> client;

        ros::NodeHandle m_global_nh;
        ros::NodeHandle m_private_nh;

        ros::Subscriber m_click_point_sub_; // 接口测试

        // CoreMove的回调
        void doneCb(const actionlib::SimpleClientGoalState &state, const upros_message::CoreMoveResultConstPtr &result);
        void activeCb();
        void feedbackCb(const upros_message::CoreMoveFeedbackConstPtr &feedback);
        void clickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg);
    };
}

#endif
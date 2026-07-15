#ifndef CORE_MOVE_H
#define CORE_MOVE_H

#include <actionlib/server/simple_action_server.h>
#include <upros_message/CoreMoveAction.h>
#include "common.h"
#include "go_straight_planner.h"

namespace core_move
{
    class CoreMove
    {
    public:
        CoreMove(boost::shared_ptr<tf2_ros::Buffer> tf);

        virtual ~CoreMove() {};

    private:
        actionlib::SimpleActionServer<upros_message::CoreMoveAction> *as_;
        boost::shared_ptr<tf2_ros::Buffer> tf_;
        boost::shared_ptr<core_move::BasePlanner> bp_;

        int controller_frequency_;
        ros::Publisher vel_pub_;

        ros::Subscriber scan_sub_;

        std::string cmd_vel_;

        upros_message::CoreMoveFeedback feedback_;

        upros_message::CoreMoveResult result_;

        void executeCB(const upros_message::CoreMoveGoalConstPtr &goal);

        bool executeCycle();

        void publishZeroVelocity();
    };
};

#endif // CORE_MOVE_H

#include <upros_core_move/core_move.h>
#include <upros_core_move/go_straight_planner.h>

namespace core_move
{
    CoreMove::CoreMove(boost::shared_ptr<tf2_ros::Buffer> tf) : tf_(tf),
                                                                controller_frequency_(10),
                                                                bp_(nullptr),
                                                                as_(nullptr)
    {
        ros::NodeHandle global_nh;
        ros::NodeHandle private_nh("~");

        private_nh.param("cmd_vel", cmd_vel_, std::string("/cmd_vel"));
        vel_pub_ = global_nh.advertise<geometry_msgs::Twist>(cmd_vel_, 1);
        // 声明一个action服务端
        as_ = new actionlib::SimpleActionServer<upros_message::CoreMoveAction>(
            ros::NodeHandle(), "core_move", boost::bind(&CoreMove::executeCB, this, _1), false);
        as_->start();
    }

    bool CoreMove::executeCycle()
    {
        if (bp_->isGoalReached())
        {
            publishZeroVelocity();
            return true;
        }
        geometry_msgs::Twist cmd_vel;
        if (bp_->computeVelocityCommands(cmd_vel))
        {
            vel_pub_.publish(cmd_vel);
        }
        return false;
    }

    // 单个action回调
    void CoreMove::executeCB(const upros_message::CoreMoveGoalConstPtr &goal)
    {
        ros::Rate r(controller_frequency_);
        // 规划器参考map到base_link的坐标变换
        if (goal->cmd == 1)
        {
            bp_ = boost::shared_ptr<GoStraightPlanner>(new GoStraightPlanner(tf_, goal->target_pose, "map", "base_link"));
        }
        ros::NodeHandle n;
        while (n.ok())
        {
            // 有抢占(cancel)，直接return
            if (as_->isPreemptRequested())
            {
                return;
            }
            bool done = executeCycle();
            // 点到达了，直接return
            if (done)
            {
                result_.final_result = 1;
                as_->setSucceeded(result_, "Goal reached.");
                return;
            }
            else
            {
                // 进度回调
                feedback_.runing = 1;
                as_->publishFeedback(feedback_);
            }
        }
        as_->setAborted(result_, "Aborting on goal.");
        return;
    }

    void CoreMove::publishZeroVelocity()
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }
};

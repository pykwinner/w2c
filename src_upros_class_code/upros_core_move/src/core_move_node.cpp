#include <ros/ros.h>
#include <upros_core_move/core_move.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "core_move_node");

    boost::shared_ptr<tf2_ros::Buffer> buffer(new tf2_ros::Buffer(ros::Duration(5)));
    tf2_ros::TransformListener tf(*buffer);

    core_move::CoreMove core_move(buffer);

    ros::spin();

    return 0;
}

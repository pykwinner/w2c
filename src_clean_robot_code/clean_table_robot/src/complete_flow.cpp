#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    double put_x_1 = 1.92;  // 桌子左侧垃圾桶x
    double put_y_1 = -1.83; // 桌子左侧垃圾桶y

    double put_x_2 = 1.92;  // 桌子右侧垃圾桶x
    double put_y_2 = -1.83; // 桌子右侧垃圾桶y

    move_base_msgs::MoveBaseGoal goal;
    tf2::Quaternion quaternion;

    /*
    *****************************第一个点,左下方，朝前，放到左侧垃圾桶****************************************************
    */
    double grab_x_1 = 1.92;  // 餐余垃圾第一个点x
    double grab_y_1 = -1.83; // 餐余垃圾第一个点y
    quaternion.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = grab_x_1;
    goal.target_pose.pose.position.y = grab_y_1;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 10)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取物块
        system("roslaunch clean_table_robot arm_grab.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm，防止规划时发生碰撞
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = -0.1;
    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    // 发送放置导航点
    quaternion.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = put_x_1;
    goal.target_pose.pose.position.y = put_y_1; // 放置点位于抓取点右侧约0.4米
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 放置物块
        system("roslaunch clean_table_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    /*
    *****************************第二个点，右下方，朝前，放到右侧垃圾桶****************************************************
    */
    double grab_x_2 = 1.92;  // 餐余垃圾第二个点x
    double grab_y_2 = -1.83; // 餐余垃圾第二个点y
    quaternion.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = grab_x_1;
    goal.target_pose.pose.position.y = grab_y_1;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 10)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取物块
        system("roslaunch clean_table_robot arm_grab.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm，防止规划时发生碰撞
    vel_msg.linear.x = -0.1;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    // 发送放置导航点
    quaternion.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = put_x_2;
    goal.target_pose.pose.position.y = put_y_2;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 放置物块
        system("roslaunch clean_table_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    /*
    *****************************第三个点，左上方，向后，放到左侧垃圾桶****************************************************
    */
    double grab_x_3 = 1.92;  // 餐余垃圾第三个点x
    double grab_y_3 = -1.83; // 餐余垃圾第三个点y
    quaternion.setRPY(0, 0, 3.1415926);
    goal.target_pose.pose.position.x = grab_x_3;
    goal.target_pose.pose.position.y = grab_y_3;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 10)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取物块
        system("roslaunch clean_table_robot arm_grab.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm，防止规划时发生碰撞
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    // 发送放置导航点
    quaternion.setRPY(0, 0, 3.1415926);
    goal.target_pose.pose.position.x = put_x_1;
    goal.target_pose.pose.position.y = put_y_1;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 放置物块
        system("roslaunch clean_table_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    /*
    *****************************第四个点，右上方，向后，放到右侧垃圾桶****************************************************
    */
    double grab_x_4 = 1.92;  // 餐余垃圾第四个点x
    double grab_y_4 = -1.83; // 餐余垃圾第四个点y
    quaternion.setRPY(0, 0, 3.1415926);
    goal.target_pose.pose.position.x = grab_x_4;
    goal.target_pose.pose.position.y = grab_y_4;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约10cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 10)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取物块
        system("roslaunch clean_table_robot arm_grab.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm，防止规划时发生碰撞
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    // 发送放置导航点
    quaternion.setRPY(0, 0, 3.1415926);
    goal.target_pose.pose.position.x = put_x_2;
    goal.target_pose.pose.position.y = put_y_2;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 放置物块
        system("roslaunch clean_table_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    // 发送餐具分拣导航点 1，在下方向前
    double part_x_1 = 1.92;  // 分拣第一个点x
    double part_y_1 = -1.83; // 分拣第一个点y
    quaternion.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = part_x_1;
    goal.target_pose.pose.position.y = part_y_1;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取餐具
        system("roslaunch clean_table_robot arm_grab.launch");
        // 分拣餐具
        system("roslaunch clean_table_robot arm_part.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    // 发送餐具分拣导航点 2，在上方向后
    double part_x_2 = 1.92;  // 分拣第二个点x
    double part_y_2 = -1.83; // 分拣第二个点y
    quaternion.setRPY(0, 0, 3.1415926);
    goal.target_pose.pose.position.x = part_x_2;
    goal.target_pose.pose.position.y = part_y_2;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 先前进一小段,大约18cm
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.1;
        int count = 0;
        ros::Rate loop_rate(10);
        while (ros::ok() && count < 18)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        // 抓取餐具
        system("roslaunch clean_table_robot arm_grab.launch");
        // 分拣餐具
        system("roslaunch clean_table_robot arm_part.launch");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }
    // 先后退一小段,大约30cm
    vel_msg.linear.x = -0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    // 发送返回导航点
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
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

#!/usr/bin/env python3

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
       
        # 初始化ROS节点
        rospy.init_node('moveit_pose_demo')
               
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm_group')
               
        # 获取终端link的名称，就是机械臂抓手的位置
        planning_frame = arm.get_planning_frame()
        rospy.loginfo("Planning frame: %s", planning_frame)
        eef_link = arm.get_end_effector_link()
        rospy.loginfo("End effector link: %s", eef_link)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
                       
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(1.0)
        arm.set_max_velocity_scaling_factor(1.0)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于arm_base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = planning_frame
        target_pose.header.stamp = rospy.Time.now()    
        target_pose.pose.position.x = 0.134
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.172
        # 这里的四元数是绕(0.70692/sin(θ/2)轴旋转，角度为arccos(0.70729)*2
        target_pose.pose.orientation.x = 0.606
        target_pose.pose.orientation.y = 0.644
        target_pose.pose.orientation.z = 0.374
        target_pose.pose.orientation.w = 0.282
       
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
       
        # 设置机械臂终端运动的目标位姿
        rospy.loginfo("Moving to target_pose ...")
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        rospy.loginfo("Moving to pose: Home")
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()
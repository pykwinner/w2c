#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, Twist
from nav_msgs.msg import Odometry, Path

class OdomTracker:
    def __init__(self):
        # 初始化ROS节点和发布者
        rospy.init_node('odom_tracker')
        self.path_pub = rospy.Publisher('/odom_path', Path, queue_size=10)
        # 设置默认的机器人位姿
        self.current_pose = Odometry()
        self.current_pose.pose.pose.position.x = 0.0
        self.current_pose.pose.pose.position.y = 0.0
        self.current_pose.pose.pose.orientation.w = 1.0

    def odom_callback(self, msg):
        # 更新机器人的位姿
        self.current_pose = msg
        # 将机器人的位姿添加到轨迹中
        path = Path()
        path.header = self.current_pose.header
        path.poses.append(self.current_pose.pose.pose)
        # 发布轨迹信息
        self.path_pub.publish(path)

if __name__ == '__main__':
    try:
        # 创建OdomTracker对象
        tracker = OdomTracker()
        # 订阅/odom话题，并设置回调函数为odom_callback
        rospy.Subscriber('/odom', Odometry, tracker.odom_callback)
        # 进入ROS循环
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

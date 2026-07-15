#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scanCallback(scan):
    # 获取激光点的个数
    num_points = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
    rospy.loginfo("Number of points: %d", num_points)
    # 获取第一个激光点的距离
    first_range = scan.ranges[0]
    rospy.loginfo("First range: %f", first_range)
    # 获取中间激光点的距离
    middle_range = scan.ranges[int(num_points/2)]
    rospy.loginfo("Middle range: %f", middle_range)
    # 获取四分之一激光点的距离
    half_middle_range = scan.ranges[int(num_points/4)]
    rospy.loginfo("Half Middle range: %f", half_middle_range)

rospy.init_node('laser_scan_listener', anonymous=True)
sub = rospy.Subscriber('/scan', LaserScan, scanCallback)
rospy.spin()
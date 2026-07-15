#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def scanCallback(scan):
    num_points = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
    first_range = scan.ranges[0]
    last_range = scan.ranges[num_points - 1]
    mean_range = (first_range + last_range) / 2.0
    rospy.loginfo("Mean range: %f", mean_range)

    cmd_vel_msg = Twist()
    if mean_range >= 8.0:
        cmd_vel_msg.linear.x = 0.0
    else:
        cmd_vel_msg.linear.x = 0.3
    cmd_vel_pub.publish(cmd_vel_msg)

rospy.init_node('laser_scan_listener', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, scanCallback)
rospy.spin()
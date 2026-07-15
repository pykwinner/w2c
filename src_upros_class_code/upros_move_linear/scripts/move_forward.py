#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('velocity_publisher', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(10)  # 10Hz，即每 100 毫秒一次

vel_msg = Twist()
vel_msg.linear.x = 0.3
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

count = 0

while not rospy.is_shutdown() and count < 50:  # 50 * 100ms = 5000ms = 5s
    pub.publish(vel_msg)
    rate.sleep()
    count += 1

vel_msg.linear.x = 0.0
pub.publish(vel_msg)
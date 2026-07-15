#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

def rangeCallback1(msg):
    rospy.loginfo("Distance Front: %f", msg.range)
    if msg.range < 0.4:
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -0.2
        cmd_vel_pub.publish(cmd_vel_msg)
    else:
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_pub.publish(cmd_vel_msg)

def rangeCallback2(msg):
    rospy.loginfo("Distance Left: %f", msg.range)

def rangeCallback3(msg):
    rospy.loginfo("Distance Right: %f", msg.range)

def rangeCallback4(msg):
    rospy.loginfo("Distance Back: %f", msg.range)

rospy.init_node('range_subscriber', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub_1 = rospy.Subscriber('/front_sonar', Range, rangeCallback1)
sub_2 = rospy.Subscriber('/left_sonar', Range, rangeCallback2)
sub_3 = rospy.Subscriber('/right_sonar', Range, rangeCallback3)
sub_4 = rospy.Subscriber('/back_sonar', Range, rangeCallback4)
rospy.spin()
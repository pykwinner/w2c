#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

def rangeCallback1(msg):
    rospy.loginfo("Distance Front: %f", msg.range)

def rangeCallback2(msg):
    rospy.loginfo("Distance Left: %f", msg.range)

def rangeCallback3(msg):
    rospy.loginfo("Distance Right: %f", msg.range)

def rangeCallback4(msg):
    rospy.loginfo("Distance Back: %f", msg.range)

if __name__ == '__main__':
    rospy.init_node('range_subscriber', anonymous=True)
    sub_1 = rospy.Subscriber('/front_sonar', Range, rangeCallback1)
    sub_2 = rospy.Subscriber('/left_sonar', Range, rangeCallback2)
    sub_3 = rospy.Subscriber('/right_sonar', Range, rangeCallback3)
    sub_4 = rospy.Subscriber('/back_sonar', Range, rangeCallback4)
    rospy.spin()
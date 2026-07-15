#!/usr/bin/env python3

import rospy

rospy.init_node('py_node', anonymous=True)

print("Hello ROS Python!!!")

rospy.spin()

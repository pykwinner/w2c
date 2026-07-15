#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(imu_msg):
    linear_acceleration = imu_msg.linear_acceleration
    angular_velocity = imu_msg.angular_velocity
    orientation = imu_msg.orientation
    rospy.loginfo(f"Linear acceleration: x={linear_acceleration.x}, y={linear_acceleration.y}, z={linear_acceleration.z}")
    rospy.loginfo(f"Angular velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")
    rospy.loginfo(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

rospy.init_node('imu_listener', anonymous=True)
imu_sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
rospy.spin()
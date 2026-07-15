#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import tf2_ros
import math
import angles

current_yaw = 0.0

def quaternion_to_euler(quaternion):
    """
    将四元数转换为欧拉角（滚动角、俯仰角、偏航角）。
    :param quaternion: 四元数，(x, y, z, w)格式。
    :return: 欧拉角，(roll, pitch, yaw)格式，弧度制。
    """
    qx, qy, qz, qw = quaternion
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(2 * (qw * qy - qz * qx))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw

def imu_callback(imu_msg):
    global current_yaw
    linear_acceleration = imu_msg.linear_acceleration
    angular_velocity = imu_msg.angular_velocity
    orientation = imu_msg.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = quaternion_to_euler(quaternion)
    current_yaw = euler[2]
    rospy.loginfo(f"Linear acceleration: x={linear_acceleration.x}, y={linear_acceleration.y}, z={linear_acceleration.z}")
    rospy.loginfo(f"Angular velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")
    rospy.loginfo(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

rospy.init_node('imu_listener', anonymous=True)
imu_sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
rotate_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

start_yaw = current_yaw
got_180 = False

while not rospy.is_shutdown() and not got_180:
    if not got_180:
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.5
        cmd_vel.linear.x = 0.0
        rotate_pub.publish(cmd_vel)
        distance_to_180 = abs(angles.shortest_angular_distance(current_yaw, start_yaw + math.pi))
        if distance_to_180 < 0.1:
            got_180 = True
            cmd_vel.angular.z = 0.0
            rotate_pub.publish(cmd_vel)
    rospy.sleep(0.05)
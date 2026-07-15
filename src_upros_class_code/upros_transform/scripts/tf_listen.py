#!/usr/bin/env python3

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform("odom", "base_link", rospy.Time(0))
            x, y, z = trans
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            rospy.loginfo("Base link in odom coordinate system: x = %f, y = %f, z = %f, roll = %f, pitch = %f, yaw = %f", x, y, z, roll, pitch, yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF Exception occurred")
            rospy.sleep(1.0)
        rate.sleep()

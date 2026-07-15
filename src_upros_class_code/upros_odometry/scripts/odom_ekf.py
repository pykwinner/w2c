#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomEKF():
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_ekf', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.ekf_pub = rospy.Publisher('output', Odometry, queue_size=5)

        # Wait for the /odom_combined topic to become available
        rospy.wait_for_message('input', PoseWithCovarianceStamped)

        # Subscribe to the /odom_combined topic for pose
        rospy.Subscriber('input', PoseWithCovarianceStamped, self.pub_ekf_odom)
        
        # Subscribe to the /wheel_odom topic for twist
        rospy.Subscriber('/wheel_odom', Odometry, self.update_twist)

        self.current_twist = None

        rospy.loginfo("Publishing combined odometry on")

    def update_twist(self, msg):
        self.current_twist = msg.twist

    def pub_ekf_odom(self, msg):
        if self.current_twist is None:
#            rospy.logwarn("No twist data received yet.")
            return

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom_combined'
        odom.child_frame_id = 'base_footprint'
        odom.pose = msg.pose
        odom.twist = self.current_twist

        self.ekf_pub.publish(odom)

if __name__ == '__main__':
    try:
        OdomEKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception: %s", str(e))

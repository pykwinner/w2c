#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2

def image_callback(msg):
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Image", img)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr("cv_bridge exception: %s" % e)

if __name__ == '__main__':
    rospy.init_node('image_subscriber')
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    rospy.spin()
    cv2.destroyWindow("Image")

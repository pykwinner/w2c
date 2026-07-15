#!/usr/bin/env python3  
  
import rospy  
import cv2  
import numpy as np
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError  

from dynamic_reconfigure.server import Server
from upros_cv.cfg import params_colorConfig


class ImageSubscriberNode:  
    def __init__(self):  
  
        rospy.init_node('image_subscriber_node', anonymous=True)  
        
        self.srv = Server(params_colorConfig, self.callback)
        
        self.hmin = 0
        self.smin = 0
        self.vmin = 0
        self.hmax = 180
        self.smax = 255
        self.vmax = 255

        self.bridge = CvBridge()  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.image_mask_pub = rospy.Publisher('/image_mask', Image, queue_size=10)
        self.image_result_pub = rospy.Publisher('/image_result', Image, queue_size=10)

    def callback(self,config, level):
        rospy.loginfo("Reconfigure Request: %d %d %d %d %d %d",
                      config.HSV_H_MIN, 
                      config.HSV_S_MIN,
                      config.HSV_V_MIN,
                      config.HSV_H_MAX,
                      config.HSV_S_MAX,
                      config.HSV_V_MAX)
        self.hmin = config.HSV_H_MIN
        self.smin = config.HSV_S_MIN
        self.vmin = config.HSV_V_MIN
        self.hmax = config.HSV_H_MAX
        self.smax = config.HSV_S_MAX
        self.vmax = config.HSV_V_MAX
        return config
        
    # 定义 ROS 回调图像函数
    def image_callback(self, msg):  
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            src = cv_image.copy()
            self.update_frame(src, self.hmin, self.hmax, self.smin, self.smax, self.vmin, self.vmax)         
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return    

    def update_frame(self, img, h_min, h_max, s_min, s_max, v_min, v_max):
        src_frame = img
        result = src_frame
        hsv_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2HSV)

        low_color = np.array([h_min, s_min, v_min])
        high_color = np.array([h_max, s_max, v_max])
        mask_color = cv2.inRange(hsv_frame, low_color, high_color)
        mask_color = cv2.medianBlur(mask_color, 7)
        
        ros_mask_image = self.bridge.cv2_to_imgmsg(mask_color, "8UC1")
        self.image_mask_pub.publish(ros_mask_image)
        
        cnts, hierarchy = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c_list = []
        for cnt in cnts:
            (x, y, w, h) = cv2.boundingRect(cnt)
            c_list.append((x, y, w, h))
            
        if len(c_list) == 1:
            (x, y, w, h) = c_list[0]
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)  
            self.target_x = x + w/2
            self.target_y = y + h/2
            offset_x = self.target_x - img.shape[1] / 2
            offset_y = self.target_y - img.shape[0] / 2
        ros_result_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
        self.image_result_pub.publish(ros_result_image)
  
    def spin(self):  
        # 让ROS节点保持运行，直到被关闭  
        rospy.spin()  
  

  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass

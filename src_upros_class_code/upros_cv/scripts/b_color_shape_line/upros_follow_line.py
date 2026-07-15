#!/usr/bin/env python3  
  
import rospy  
import cv2  
import numpy as np
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
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
        
        self.enable_move = False

        self.bridge = CvBridge()  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.enable_sub = rospy.Subscriber('/enable_move', Int16, self.enable_callback)
        self.image_mask_pub = rospy.Publisher('/image_mask', Image, queue_size=10)
        self.image_result_pub = rospy.Publisher('/image_result', Image, queue_size=10)
        # 创建一个发布者对象，发布移动速度
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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
    
    def enable_callback(self, msg):
        if(msg.data == 1):
            self.enable_move = True
               
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
        h, w, d = src_frame.shape
        search_top = 5*h//6
        mask_color[0:search_top, 0:w] = 0
        ros_mask_image = self.bridge.cv2_to_imgmsg(mask_color, "8UC1")
        self.image_mask_pub.publish(ros_mask_image)
        
        # 求截取区域的代数中心，并在此中心画一个红色实心小圆来代表它
        M = cv2.moments(mask_color)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            err = cx - w/2
            linear_x = 0.2
            angular_z = -float(err) / 100
            self.move_up(linear_x, 0.0, angular_z) 
            cv2.circle(result, (cx, cy), 20, (0,0,255), -1)
        else: 
            self.move_up(0.0, 0.0, 0.0)    
        ros_result_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
        self.image_result_pub.publish(ros_result_image)

    # 生成移动命令
    def move_up(self, x, y, th):
        t = Twist()
        t.linear.x = x
        t.linear.y = y
        t.angular.z = th
        if (self.enable_move):
            self.cmd_pub.publish(t)

  
    def spin(self):  
        # 让ROS节点保持运行，直到被关闭  
        rospy.spin()  
  

  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass



      




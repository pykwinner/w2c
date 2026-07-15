#!/usr/bin/env python3  
  
import rospy  
import cv2  
import numpy as np
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError  
  
class ImageSubscriberNode:  
    def __init__(self):  
        # 初始化ROS节点  
        rospy.init_node('image_subscriber_node', anonymous=True)  
        
        #初始化颜色参数
        self.line_h_min = rospy.get_param('~line_h_min', '20')
        self.line_h_max = rospy.get_param('~line_h_max', '30')
        self.line_s_min = rospy.get_param('~line_s_min', '0')
        self.line_s_max = rospy.get_param('~line_s_max', '0')
        self.line_v_min = rospy.get_param('~line_v_min', '0')
        self.line_v_max = rospy.get_param('~line_v_max', '0')

        self.enable_move = False

        # 设置一个窗口来显示图像  
        self.window_name = "USB Camera Image"  
        cv2.namedWindow(self.window_name) 
 
        # 创建CvBridge对象，用于将ROS图像消息转换为OpenCV图像  
        self.bridge = CvBridge()  
        # 创建一个订阅者对象，订阅/usb_cam/image_raw话题  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        # 创建一个订阅者对象,start move
        self.cmd_sub = rospy.Subscriber('/command', String, self.cmd_callback) 
        # 创建一个发布者对象，发布移动速度
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def cmd_callback(self, msg):  
        if(msg.data == 'start'):
            self.enable_move = True
  
    def image_callback(self, msg):  
        # 使用CvBridge将ROS图像消息转换为OpenCV图像  
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            src = cv_image.copy()
            self.update_frame(src, self.line_h_min, self.line_h_max, self.line_s_min, self.line_s_max, self.line_v_min, self.line_v_max)
            cv2.imshow("result", src)            

        except CvBridgeError as e:  
            rospy.logerr(e)  
            return    

        # 显示图像  
        cv2.imshow(self.window_name, cv_image)  
        key = cv2.waitKey(1) & 0xFF  # 等待1ms，并获取按键信息  
        if key == ord('q'):  # 如果按下'q'键，则退出循环  
            rospy.signal_shutdown('User requested shutdown by pressing "q"')
    
    
    #根据颜色输出坐标
    def update_frame(self, img, h_min, h_max, s_min, s_max, v_min, v_max):
        src_frame = img
        result = src_frame
        #颜色空间转换
        self.hsv_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2HSV)
        #二值化
        low_color = np.array([h_min, s_min, v_min])
        high_color = np.array([h_max, s_max, v_max])
        mask_color = cv2.inRange(self.hsv_frame, low_color, high_color)
        #滤波
        mask_color = cv2.medianBlur(mask_color, 7)
        h, w, d = src_frame.shape
        search_top = 5*h//6
        mask_color[0:search_top, 0:w] = 0
        cv2.imshow("mask", mask_color)
        # 求截取区域的代数中心，并在此中心画一个红色实心小圆来代表它
        M = cv2.moments(mask_color)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img, (cx, cy), 20, (0,0,255), -1)
            err = cx - w/2
            linear_x = 0.2
            angular_z = -float(err) / 100
            self.move_up(linear_x, 0.0, angular_z) 
        else: 
            self.move_up(0.0, 0.0, 0.0)          

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
  
    def cleanup(self):  
        # 关闭OpenCV窗口  
        cv2.destroyAllWindows()  
    
    def nothing(self):
        pass
  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        rospy.on_shutdown(node.cleanup)  # 当ROS节点关闭时，清理OpenCV窗口  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass

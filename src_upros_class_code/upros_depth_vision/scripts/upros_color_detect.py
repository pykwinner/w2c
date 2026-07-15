#!/usr/bin/env python3  
  
import rospy  
import cv2  
import numpy as np
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError  
  
class ImageSubscriberNode:  
    def __init__(self):  
        # 初始化ROS节点  
        rospy.init_node('image_subscriber_node', anonymous=True)  
        #初始化颜色参数
        self.target_H = 0
        self.target_S = 0
        self.target_V = 0
        self.gray_frame = None
        self.hsv_frame = None
        self.target_x = None
        self.target_y = None
        self.is_detect = False
        # 设置一个窗口来显示图像  
        self.window_name = "USB Camera Image"  
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL) 
        cv2.createTrackbar("H_MIN","USB Camera Image",35,180,self.nothing)
        cv2.createTrackbar("H_MAX","USB Camera Image",40,180,self.nothing)
        cv2.createTrackbar("S_MIN","USB Camera Image",100,255,self.nothing)
        cv2.createTrackbar("S_MAX","USB Camera Image",115,255,self.nothing)
        cv2.createTrackbar("V_MIN","USB Camera Image",180,255,self.nothing)
        cv2.createTrackbar("V_MAX","USB Camera Image",190,255,self.nothing)
        cv2.createTrackbar("X_STRIDE","USB Camera Image",40,649,self.nothing)
        cv2.createTrackbar("Y_STRIDE","USB Camera Image",40,480,self.nothing)
        cv2.createTrackbar("AREA_STRIDE","USB Camera Image",1000,10000,self.nothing)
        cv2.createTrackbar("PIXEL_STRIDE","USB Camera Image",1000,10000,self.nothing)
        cv2.setMouseCallback("USB Camera Image", self.mouse_click)
        
        # 创建CvBridge对象，用于将ROS图像消息转换为OpenCV图像  
        self.bridge = CvBridge()  
        
        # 创建一个订阅者对象，订阅/camera_color/image_raw话题  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.center_pub = rospy.Publisher('/image_point', Point, queue_size=10)
 
  
    def image_callback(self, msg):  
        # 使用CvBridge将ROS图像消息转换为OpenCV图像  
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            src = cv_image.copy()
            #获取HSV的阈值
            h_min = cv2.getTrackbarPos("H_MIN","USB Camera Image")
            h_max = cv2.getTrackbarPos("H_MAX","USB Camera Image")
            s_min = cv2.getTrackbarPos("S_MIN","USB Camera Image")
            s_max = cv2.getTrackbarPos("S_MAX","USB Camera Image") 
            v_min = cv2.getTrackbarPos("V_MIN","USB Camera Image")
            v_max = cv2.getTrackbarPos("V_MAX","USB Camera Image")
            x_stride = cv2.getTrackbarPos("X_STRIDE","USB Camera Image")
            y_stride = cv2.getTrackbarPos("Y_STRIDE","USB Camera Image")
            area_stride = cv2.getTrackbarPos("AREA_STRIDE","USB Camera Image")
            pixel_stride = cv2.getTrackbarPos("PIXEL_STRIDE","USB Camera Image")
            self.update_frame(src, h_min, h_max, s_min, s_max, v_min, v_max, x_stride, y_stride, area_stride, pixel_stride)
            cv2.imshow("result", src)            

        except CvBridgeError as e:  
            rospy.logerr(e)  
            return    

        # 显示图像  
        cv2.imshow(self.window_name, cv_image)  
        key = cv2.waitKey(1) & 0xFF  # 等待1ms，并获取按键信息  
        if key == ord('q'):  # 如果按下'q'键，则退出循环  
            rospy.signal_shutdown('User requested shutdown by pressing "q"')
    
    #鼠标点击回调
    def mouse_click(self, event, x, y, flags, para):
        if event == cv2.EVENT_LBUTTONDOWN:
            print ('PIX: ', x, y)
            print ('GRAY: ', self.gray_frame[y, x])
            print ('HSV: ', self.hsv_frame[y, x])
    
    #根据颜色输出坐标
    def update_frame(self, img, h_min, h_max, s_min, s_max, v_min, v_max, x_stride, y_stride, area_stride, pixel_stride):
        src_frame = img
        result = src_frame
        #颜色空间转换
        self.gray_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2GRAY)
        self.hsv_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2HSV)
        #二值化
        low_color = np.array([h_min, s_min, v_min])
        high_color = np.array([h_max, s_max, v_max])
        mask_color = cv2.inRange(self.hsv_frame, low_color, high_color)
        #滤波
        mask_color = cv2.medianBlur(mask_color, 7)
        cv2.imshow("mask", mask_color)
        #提取联通域
        cnts, hierarchy = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        c_list = []
        
        for cnt in cnts:
            num_points = len(cnt)
            (x, y, w, h) = cv2.boundingRect(cnt)
            if w < x_stride or h < y_stride :
                continue  # 排除宽和高不符合要求的轮廓
            if w*h < area_stride:
                continue #排除面积不符合要求的轮廓
            if num_points < pixel_stride:
                continue #排除像素点个数不满足要求的轮廓
            #将每个满足条件的颜色矩形存入cnt_list
            c_list.append((x, y, w, h))

        if len(c_list) == 1:
            (x, y, w, h) = c_list[0]
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)  
            self.target_x = x + w/2
            self.target_y = y + h/2
            #计算颜色中心相对于图像中心的像素距离
            offset_x = self.target_x - img.shape[1] / 2
            offset_y = self.target_y - img.shape[0] / 2
            point_msg = Point()
            point_msg.x = offset_x
            point_msg.y = offset_y
            self.center_pub.publish(point_msg)
  
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

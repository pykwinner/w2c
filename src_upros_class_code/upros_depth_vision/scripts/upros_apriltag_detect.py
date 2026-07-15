#!/usr/bin/env python3  
  
import rospy  
import cv2  
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError 
import apriltag 
  
class ImageSubscriberNode:  
    def __init__(self):  
        # 初始化ROS节点  
        rospy.init_node('image_subscriber_node', anonymous=True)  
        options = apriltag.DetectorOptions(families='tag36h11')
        self.tag_detector = apriltag.Detector(options)
        self.tag_id = 2
          
        # 创建CvBridge对象，用于将ROS图像消息转换为OpenCV图像  
        self.bridge = CvBridge()  
          
        # 创建一个订阅者对象，订阅/usb_cam/image_raw话题  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.center_pub = rospy.Publisher('/image_point', Point, queue_size=10)

        # 设置一个窗口来显示图像  
        self.window_name = "USB Camera Image"  
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  
        cv2.createTrackbar("TAG_ID","USB Camera Image",1,6,self.nothing)
  
    def image_callback(self, msg):  
        # 使用CvBridge将ROS图像消息转换为OpenCV图像  
        try: 
            #获取要识别tag的id值 
            self.tag_id = cv2.getTrackbarPos("TAG_ID","USB Camera Image")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            frame = cv_image.copy()
            gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            tags = []
            results = self.tag_detector.detect(gray_frame) 
            for result in results:
                tag_id = result.tag_id
                if(tag_id == self.tag_id):
                    top_left, top_right, bottom_right, bottom_left = result.corners
                    center_x = int((top_left[0] + bottom_right[0]) / 2) 
                    center_y = int((top_left[1] + bottom_right[1]) / 2)
                    #识别到的tag画十字标记
                    thickness = 2  # 线条粗度  
                    size = 20 # 线条长度  
                    color = (0, 0, 255)  # 红色，BGR格式  
                    # 计算十字的四个端点坐标   
                    p1 = (center_x - size, center_y)  
                    p2 = (center_x + size, center_y)  
                    p3 = (center_x, center_y - size)  
                    p4 = (center_x, center_y + size)  
                    # 在图像上画十字  
                    cv2.line(frame, p1, p2, color, thickness)  
                    cv2.line(frame, p3, p4, color, thickness)
                    #计算颜色中心相对于图像中心的像素距离
                    offset_x = center_x - cv_image.shape[1] / 2
                    offset_y = center_y - cv_image.shape[0] / 2
                    point_msg = Point()
                    point_msg.x = offset_x
                    point_msg.y = offset_y
                    self.center_pub.publish(point_msg)
                    cv2.imshow('Result', frame)
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return   
        # 显示图像  
        cv2.imshow(self.window_name, cv_image)  
        key = cv2.waitKey(1) & 0xFF  # 等待1ms，并获取按键信息  
        if key == ord('q'):  # 如果按下'q'键，则退出循环  
            rospy.signal_shutdown('User requested shutdown by pressing "q"')
  
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

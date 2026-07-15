#!/usr/bin/env python3  
  
import rospy  
import cv2  
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError 
import apriltag 
  
class ImageSubscriberNode:  
    
    def __init__(self):    
        rospy.init_node('image_subscriber_node', anonymous=True)  
        
        options = apriltag.DetectorOptions(families='tag36h11')
        self.tag_detector = apriltag.Detector(options)
        self.tag_id = 1
        
        self.bridge = CvBridge()  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback) 
        self.image_pub = rospy.Publisher('/image_result', Image, queue_size=10)


    def image_callback(self, msg):  
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            
            frame = cv_image.copy()
            gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
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
                    # 在识别到的AprilTag重心画十字  
                    cv2.line(frame, p1, p2, color, thickness)  
                    cv2.line(frame, p3, p4, color, thickness)

            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return   
        
    def spin(self):  
        rospy.spin()  
  

  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass

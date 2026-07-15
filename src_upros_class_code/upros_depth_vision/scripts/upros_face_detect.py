#!/usr/bin/env python3  
  
import rospy  
import cv2  
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError  
import sys
  
class ImageSubscriberNode:  
    def __init__(self):  
        # 初始化ROS节点  
        rospy.init_node('image_subscriber_node', anonymous=True)  
        
        #加载一个人脸检测模型
        current_path = sys.path[0]
        modelPath = current_path + "/cascades/haarcascade_frontalface_default.xml"
        print(modelPath)
        self.face_cascade = cv2.CascadeClassifier(modelPath)
        
        # 创建CvBridge对象，用于将ROS图像消息转换为OpenCV图像  
        self.bridge = CvBridge()  
          
        # 创建一个订阅者对象，订阅/usb_cam/image_raw话题  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.center_pub = rospy.Publisher('/image_point', Point, queue_size=10)
          
        # 设置一个窗口来显示图像  
        self.window_name = "USB Camera Image"  
        cv2.namedWindow(self.window_name)  
  
    def image_callback(self, msg):  
        # 使用CvBridge将ROS图像消息转换为OpenCV图像  
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            src = cv_image.copy()
            result = cv_image.copy()
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in faces:
                result = cv2.rectangle(src, (x, y), (x + w, y + h), (255, 0, 0), 2)

            if faces is not None:
                if len(faces) == 1:
                    (fx, fy, fw, fh) = faces[0]
                    target_face_x = fx + fw / 2
                    target_face_y = fy + fh / 2
                    #计算人脸中心相对于图像中心的像素距离
                    offset_x = target_face_x - cv_image.shape[1] / 2
                    offset_y = target_face_y - cv_image.shape[0] / 2
                    point_msg = Point()
                    point_msg.x = offset_x
                    point_msg.y = offset_y
                    self.center_pub.publish(point_msg)

            cv2.imshow("result", result)            
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
  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        rospy.on_shutdown(node.cleanup)  # 当ROS节点关闭时，清理OpenCV窗口  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass

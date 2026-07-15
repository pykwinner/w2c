#!/usr/bin/env python3  
  
import rospy  
import cv2  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge, CvBridgeError  
import sys
import os
  
class ImageSubscriberNode:  
    def __init__(self):  
  
        rospy.init_node('take_photo_node', anonymous=True)  
               
        #加载一个人脸检测模型
        current_path = sys.path[0]
        modelPath = current_path + "/cascades/haarcascade_frontalface_default.xml"
        self.face_cascade = cv2.CascadeClassifier(modelPath)

        #要登记的人脸的文件夹位置,如果没有此文件夹，则新创建一个
        self.person_name = rospy.get_param('~person_name', 'name_one')     
        self.dirname = current_path + "/data/" + self.person_name + "/"
        if(not os.path.isdir(self.dirname)):
            os.makedirs(self.dirname)
            
        #存储的人脸数目
        self.picture_num = 100
        self.count = 0
        
        self.bridge = CvBridge()  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.image_pub = rospy.Publisher('/image_result', Image, queue_size=10)

  
    def image_callback(self, msg):   
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            result = cv_image.copy()
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            #人脸检测函数
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in faces:
                result = cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
            #如果只识别到了一张脸，采集若干人脸图像，总共需要采集10张人脸图像
            if(len(faces) == 1):
                (x, y, w, h) = faces[0]
                face = cv2.resize(gray[y:y+h, x:x+w], (200, 200))
                
                if self.count < self.picture_num:
                        # 判断一下，10的整数倍才保存
                        if self.count // 10 == self.count / 10.0:
                            cv2.imwrite(self.dirname + '%s.pgm' % str(self.count//10), face)
                            rospy.loginfo("save success")  
                        self.count += 1
                
                elif self.count == self.picture_num:
                        rospy.loginfo("save enough photo!!!")
                        self.count += 1
                        
            ros_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
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


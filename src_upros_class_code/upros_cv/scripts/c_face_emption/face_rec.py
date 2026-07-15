#!/usr/bin/env python3  
  
import rospy  
import cv2  
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError  
import sys
import os
import numpy as np
import rospy
  
class ImageSubscriberNode:  
    def __init__(self):  
        # 初始化ROS节点  
        rospy.init_node('face_rec_node', anonymous=True)  
        
        #人脸阈值参数
        self.confidence = rospy.get_param('~confidence', '90')    
               
        #加载一个人脸检测模型
        current_path = sys.path[0]
        modelPath = current_path + "/cascades/haarcascade_frontalface_default.xml"
        self.face_cascade = cv2.CascadeClassifier(modelPath)

        #要识别的的人脸名称
        self.face_name = ""  
        self.dirname = current_path + "/data"
        if(not os.path.isdir(self.dirname)):
            os.makedirs(self.dirname)
        
        #读取人脸图片，训练一个人脸识别模型
        [names,X, Y] = self.read_images(self.dirname)
        Y = np.asarray(Y, dtype=np.int32)
        self.model = cv2.face.LBPHFaceRecognizer_create()
        self.model.train(np.asarray(X), np.asarray(Y))
        self.names = names
        rospy.logwarn("Train Finish!!!!")
        
        # 创建CvBridge对象，用于将ROS图像消息转换为OpenCV图像  
        self.bridge = CvBridge()  
        # 创建一个订阅者对象，订阅/usb_cam/image_raw话题  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        # 创建一个订阅者对象，订阅/auto_face话题，获取要识别的人脸  
        self.face_name_sub = rospy.Subscriber("/auto_face", String, self.name_callback)
        # 创建一个发布者对象，对机器人速度进行控制 
        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        # 创建一个发布者对象，把识别的结果发布出来 
        self.result_image_pub = rospy.Publisher('/image_result', Image, queue_size=10)

        self.forward_speed = 0.3
        self.turn_speed = 1.0
           
    def read_images(self, path, sz=None):
        c = 0
        X, Y = [], []
        names = []
        for dirname, dirnames, filenames in os.walk(path):
            for subdirname in dirnames:
                subject_path = os.path.join(dirname, subdirname)
                for filename in os.listdir(subject_path):
                    try:
                        if (filename == ".directory"):
                            continue
                        filepath = os.path.join(subject_path, filename)
                        im = cv2.imread(os.path.join(subject_path, filename), cv2.IMREAD_GRAYSCALE)
                        if (im is None):
                            print("image" + filepath + "is None")
                        if (sz is not None):
                            im = cv2.resize(im, sz)
                        X.append(np.asarray(im, dtype=np.uint8))
                        Y.append(c)
                    except:
                        print("unexpected error")
                        raise
                c = c + 1
                names.append(subdirname)
        return [names, X, Y]

    def name_callback(self, msg):
        if msg.data == "g1":
            self.face_name = "g1"
        if msg.data == "g2":
            self.face_name = "g2"

    def image_callback(self, msg):  
        # 使用CvBridge将ROS图像消息转换为OpenCV图像  
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            result = cv_image.copy()
            #转化为灰度图
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            #人脸检测函数
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            #如果没有识别到人脸，直接忽略
            if (faces is None):
                self.update_cmd(0,0)
                rospy.logwarn("No Person,Ignore!!!")
                return                
            if(len(faces) == 0):
                self.update_cmd(0,0)
                rospy.logwarn("Not Person,Ignore!!!")
                return 
            for (x, y, w, h) in faces:
                result = cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 0), 2)
                roi = gray[x:x + w, y:y + h]
                try:
                    #人脸归一化，统一200x200大小
                    roi = cv2.resize(roi,(200,200),interpolation=cv2.INTER_LINEAR)
                    #人脸预测
                    [p_label, p_confidence] = self.model.predict(roi)
                    #画出名字的结果和置信度
                    cv2.putText(result, self.names[p_label], (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
                    cv2.putText(result, str(p_confidence), (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
                    if p_confidence > self.confidence and self.names[p_label] == self.face_name:
                        offset_x = ((x+w) / 2 - cv_image.shape[1]/2)
                        target_area = w * h
                        linear_vel = 0
                        angular_vel = 0
                        if target_area < 100:
                            linear_vel = 0.0
                        elif target_area > 110:
                            linear_vel = 0.1
                        else:
                            linear_vel = 0.0
                        if offset_x > 0:
                            angular_vel = 0.1
                        if offset_x < 0:
                            angular_vel = -0.1
                        self.update_cmd(linear_vel,angular_vel)
                except:
                    return
            try:  
                # 将OpenCV图像转换为ROS图像消息  
                image_message = self.bridge.cv2_to_imgmsg(result, "bgr8")  
                # 发布图像消息  
                self.result_image_pub.publish(image_message)  
            except CvBridgeError as e:  
                rospy.logerr(e)         
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return     

    def update_cmd(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.speed_pub.publish(twist)
      
    def spin(self):  
        # 让ROS节点保持运行，直到被关闭  
        rospy.spin()  
  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass
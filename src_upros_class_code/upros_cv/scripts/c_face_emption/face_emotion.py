#!/usr/bin/env python3

import rospy
import dlib                     
import numpy as np             
import cv2           
import sys

from sensor_msgs.msg import Image  
from cv_bridge import CvBridge, CvBridgeError


class Face_Emotion():

    def __init__(self):
        rospy.init_node('face_emotion_node', anonymous=True) 
        
        self.detector = dlib.get_frontal_face_detector()
        modelPath = sys.path[0] + "/shape_predictor_68_face_landmarks.dat"
        self.predictor = dlib.shape_predictor(modelPath)

        self.bridge = CvBridge()  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.image_pub = rospy.Publisher('/image_result', Image, queue_size=10)
        
    def update_frame(self, frame):
        result = frame.copy()
        # 取灰度
        img_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # 使用人脸检测器检测每一帧图像中的人脸。并返回人脸数rects
        faces = self.detector(img_gray, 0)
        # 待会要显示在屏幕上的字体
        font = cv2.FONT_HERSHEY_SIMPLEX
        # 眉毛直线拟合数据缓冲
        line_brow_x = []
        line_brow_y = []
        # 如果检测到1人脸
        if len(faces) == 1:
            # 对每个人脸都标出68个特征点
            for i in range(len(faces)):
                # enumerate方法同时返回数据对象的索引和数据，k为索引，d为faces中的对象
                for k, d in enumerate(faces):
                    # 用红色矩形框出人脸
                    cv2.rectangle(result, (d.left(), d.top()), (d.right(), d.bottom()), (0, 0, 255))
                    # 计算人脸框边长
                    self.face_width = d.right() - d.left()

                    # 使用预测器得到68点数据的坐标
                    shape = self.predictor(result, d)
                    # 圆圈显示每个特征点
                    for i in range(68):
                        cv2.circle(result, (shape.part(i).x, shape.part(i).y), 2, (0, 255, 0), -1, 8)
                        # 分析任意n点的位置关系来作为表情识别的依据
                        mouth_width = (shape.part(54).x - shape.part(48).x) / self.face_width  # 嘴巴咧开程度
                        mouth_higth = (shape.part(66).y - shape.part(62).y) / self.face_width  # 嘴巴张开程度

                        # 通过两个眉毛上的10个特征点，分析挑眉程度和皱眉程度
                        brow_sum = 0  # 高度之和
                        frown_sum = 0  # 两边眉毛距离之和
                        for j in range(17, 21):
                            brow_sum += (shape.part(j).y - d.top()) + (shape.part(j + 5).y - d.top())
                            frown_sum += shape.part(j + 5).x - shape.part(j).x
                            line_brow_x.append(shape.part(j).x)
                            line_brow_y.append(shape.part(j).y)

                        # 计算眉毛的倾斜程度
                        tempx = np.array(line_brow_x)
                        tempy = np.array(line_brow_y)
                        z1 = np.polyfit(tempx, tempy, 1)  # 拟合成一次直线
                        self.brow_k = -round(z1[0], 3)  # 拟合出曲线的斜率和实际眉毛的倾斜方向是相反的

                        brow_hight = (brow_sum / 10) / self.face_width  # 眉毛高度占比
                        brow_width = (frown_sum / 5) / self.face_width  # 眉毛距离占比

                        # 眼睛睁开程度
                        eye_sum = (shape.part(41).y - shape.part(37).y + shape.part(40).y - shape.part(38).y +
                                   shape.part(47).y - shape.part(43).y + shape.part(46).y - shape.part(44).y)
                        eye_hight = (eye_sum / 4) / self.face_width

                        # 分情况讨论
                        # 张嘴，可能是开心或者惊讶
                        if round(mouth_higth >= 0.03):
                            if eye_hight >= 0.056:
                                cv2.putText(result, "amazing", (d.left(), d.bottom() + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                            (0, 0, 255), 2, 4)
                            else:
                                cv2.putText(result, "happy", (d.left(), d.bottom() + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                            (0, 0, 255), 2, 4)
                                
                        # 没有张嘴，可能是正常和生气
                        else:
                            if self.brow_k <= -0.3:
                                cv2.putText(result, "angry", (d.left(), d.bottom() + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                            (0, 0, 255), 2, 4)
                            else:
                                cv2.putText(result, "nature", (d.left(), d.bottom() + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                            (0, 0, 255), 2, 4)
        else:
            # 没有检测到1人脸
            cv2.putText(result, "Not 1 Face", (20, 50), font, 1, (0, 0, 255), 1, cv2.LINE_AA)
        return result

    def image_callback(self, msg):  
        
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            src = cv_image.copy()
            result = self.update_frame(src)
            ros_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
            self.image_pub.publish(ros_image)      
              
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return     

    def spin(self):  
        # 让ROS节点保持运行，直到被关闭  
        rospy.spin()  
  



if __name__ == "__main__":
    my_face = Face_Emotion()
    my_face.spin()
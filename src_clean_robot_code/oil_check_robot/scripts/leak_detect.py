#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tts_player import TTS

class ImageViewer:
    def __init__(self):
        # 初始化ros node
        rospy.init_node('leak_detection', anonymous=True)

        model_path = "/home/bcsh/Documents/Models/tts/vits-zh-hf-fanchen-C"

        self.tts_player = TTS(model_dir=model_path, debug=False)
        
        # 创建bridge对象以便于转化msg和opencv images
        self.bridge = CvBridge()
        
        # 订阅主题 /usb_cam/image_raw
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        # 服务回调
        self.service = rospy.Service("/leak_detect", Empty, self.handle_detect_service)

        # 语音播报用
        self.voice_pub = rospy.Publisher('/robot_voice/llm/result', String, queue_size=10)

        # 黑色像素计数器
        self.black_pixel_count = 0
        
        # 设置关闭窗口事件监听
        cv2.startWindowThread()
    
    def handle_detect_service(self, req):
        # 保存图像
        cv2.imwrite("/home/bcsh/leak.jpg", self.result_image)
        # 语音播报
        self.tts_player.generate_and_play('漏油了请注意')
        return 0

    def callback(self,data):
        try:
            self.black_pixel_count = 0
            # 使用桥将ros msg转化为opencv image
            frame = self.bridge.imgmsg_to_cv2(data,"bgr8")

            # 调整尺寸
            img_resized = cv2.resize(frame,(640,400))
            img_resized = img_resized[200:400,0:640]

            # 将图像从BGR转换到HSV
            hsv_img = cv2.cvtColor(img_resized,cv2.COLOR_BGR2HSV)

            # 定义黄色的HSV范围
            lower_yellow = (0, 168, 141)
            upper_yellow = (42, 255, 255)

            # 进行颜色阈值处理，得到黄色区域的掩码图并进行中值滤波去除噪点
            yellow_mask = cv2.inRange(hsv_img,lower_yellow,upper_yellow)
            yellow_mask = cv2.medianBlur(yellow_mask, 9)
            
            # 黄色区域的掩码图
            cv2.imshow("yellow_mask",yellow_mask)

            # 查找黄色区域内的轮廓
            contours,_=cv2.findContours(yellow_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            # 创建一个和原图像大小相同的全白图像
            self.result_image=img_resized.copy()           
          
            # 遍历轮廓，将轮廓内的区域从原图像复制到结果图像
            for cnt in contours:
                x,y,w,h=cv2.boundingRect(cnt)
                
                # 抠出黄色连通域
                roi_frame=img_resized[y:y+h,x:x+w]
                
                # 展示黄色区域
                cv2.imshow("roi_frame",roi_frame)
                
                # 在黄色roi区域中，找到黑色区域
                lower_black=(40,0,0)
                upper_black=(142,64,80)

                # 二值化并中值滤波
                black_mask=cv2.inRange(roi_frame,lower_black,upper_black)
                black_mask = cv2.medianBlur(black_mask, 9)
                
                # 查找黑色区域内的轮廓
                black_cnts,_=cv2.findContours(black_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                
                # 在结果图像上，用红色画笔，画出黑色轮廓
                cv2.drawContours(roi_frame,black_cnts,-1,(0,0,255),2)
                
                # 遍历黑色轮廓，将轮廓内的区域从原图像复制到结果图像
                for bcnt in black_cnts:
                    bx,by,bw,bh=cv2.boundingRect(bcnt)
                    
                    black_roi=roi_frame[by:by+bh,bx:bx+bw]
                    self.result_image[y+by:y+by+bh,x+bx:x+bx+bw]=black_roi
                    
                    # 计算轮廓内的黑色像素数量，叠加到一起
                    contour_area = w * h
                    self.black_pixel_count+=contour_area
            
            # 如果黄色区域内的黑色超过阈值，发生漏油，打出Leak字样
            if self.black_pixel_count>700:
                cv2.putText(self.result_image,'Leak',(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            else:
                cv2.putText(self.result_image,'UnLeak',(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)

            # 显示结果图像
            cv2.imshow("Result",self.result_image)
            cv2.waitKey(1)  

        except CvBridgeError as e:
            print(e)

if __name__=='__main__':
    viewer=ImageViewer()
    try:
        rospy.spin()
    finally:
        cv2.destroyAllWindows()

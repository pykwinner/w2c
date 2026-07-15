#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
import math

from sensor_msgs.msg import Image  
from cv_bridge import CvBridge, CvBridgeError  


class ImageSubscriberNode:  
    def __init__(self):  
        rospy.init_node('take_photo_node', anonymous=True)  
        
        self.draw = mp.solutions.drawing_utils
        
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.75,
            min_tracking_confidence=0.75)

        self.bridge = CvBridge()  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.image_pub = rospy.Publisher('/image_result', Image, queue_size=10)

    def image_callback(self, msg):   
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            result = cv_image.copy()
            hands_landmarks = self.findHind(result, self.hands, self.draw)
            if hands_landmarks:
                resultNumber = self.detectNumber(hands_landmarks, result)
                if (resultNumber >= 0):
                    cv2.putText(result, str(resultNumber), (150, 150), 19, 5, (255, 0, 255), 5, cv2.LINE_AA)
                    if(resultNumber == 5):
                        print("Move Forward")
                    elif (resultNumber == 0):
                        print("Move backward")
                else:
                    cv2.putText(result, "NO NUMBER", (150, 150), 20, 1, (0, 0, 255))
            ros_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
            self.image_pub.publish(ros_image)         
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return    
            
    def findHind(self, img, hands, draw):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 转换为RGB
	
        handlmsstyle = draw.DrawingSpec(color=(0, 0, 255), thickness=5)
        handconstyle = draw.DrawingSpec(color=(0, 255, 0), thickness=5)

        results = hands.process(imgRGB)
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                draw.draw_landmarks(img, handLms, mp.solutions.hands.HAND_CONNECTIONS, handlmsstyle, handconstyle)
        return results.multi_hand_landmarks


    def detectNumber(self, hand_landmarks, img):
        """
        :param hand_landmarks: 手势特征
        :param img: 实时图像
        :return: 返回识别到的数字，如果没有则返回-1
        """

        h, w, c = img.shape

        myhand = hand_landmarks[0]
        hand_landmark = myhand.landmark
        thumb_tip_id = 4  # 大拇指指尖
        index_finger_tip_id = 8  # 食指指尖
        middle_finger_tip_id = 12  # 中指指尖
        ring_finger_tip_id = 16  # 无名指指尖
        pinky_finger_tip_id = 20  # 小指指尖
        pinky_finger_mcp_id = 17  # 小指指根（用于判断4和5）
        wrist_id = 0  # 手腕（用于识别数字6）

        # 提取y坐标
        thumb_tip_y = hand_landmark[thumb_tip_id].y * h
        index_tip_y = hand_landmark[index_finger_tip_id].y * h
        middle_tip_y = hand_landmark[middle_finger_tip_id].y * h
        ring_tip_y = hand_landmark[ring_finger_tip_id].y * h
        pinky_tip_y = hand_landmark[pinky_finger_tip_id].y * h
        pinky_mcp_y = hand_landmark[pinky_finger_mcp_id].y * h
        wrist_y = hand_landmark[wrist_id].y * h

        # 提取x坐标
        thumb_tip_x = hand_landmark[thumb_tip_id].x * w
        index_tip_x = hand_landmark[index_finger_tip_id].x * w
        middle_tip_x = hand_landmark[middle_finger_tip_id].x * w
        ring_tip_x = hand_landmark[ring_finger_tip_id].x * w
        pinky_tip_x = hand_landmark[pinky_finger_tip_id].x * w
        pinky_mcp_x = hand_landmark[pinky_finger_mcp_id].x * w
        wrist_x = hand_landmark[wrist_id].x * w

        dist_thumb2wrist = math.sqrt((thumb_tip_x - wrist_x)**2 + (thumb_tip_y - wrist_y)**2)
        dist_index2wrist = math.sqrt((index_tip_x - wrist_x) ** 2 + (index_tip_y - wrist_y) ** 2)
        dist_middle2wrist = math.sqrt((middle_tip_x - wrist_x) ** 2 + (middle_tip_y - wrist_y) ** 2)
        dist_ring2wrist = math.sqrt((ring_tip_x - wrist_x) ** 2 + (ring_tip_y - wrist_y) ** 2)
        dist_pinky2wrist = math.sqrt((pinky_tip_x - wrist_x) ** 2 + (pinky_tip_y - wrist_y) ** 2)
        dist_pinky_mcp2wrist = math.sqrt((thumb_tip_x - pinky_mcp_x)**2 + (thumb_tip_y - pinky_mcp_y)**2)

        # 相当于取dist_thumb2wrist_ratio == 1
        dist_index2wrist_ratio = dist_index2wrist / dist_thumb2wrist
        dist_middle2wrist_ratio = dist_middle2wrist / dist_thumb2wrist
        dist_ring2wrist_ratio = dist_ring2wrist / dist_thumb2wrist
        dist_pinky2wrist_ratio = dist_pinky2wrist / dist_thumb2wrist
        dist_pinky_mcp2wrist_ratio = dist_pinky_mcp2wrist / dist_thumb2wrist

        if dist_index2wrist_ratio < 1.9 and dist_middle2wrist_ratio < 1.8 and dist_ring2wrist_ratio < 1.6 and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 0
        elif 2.0 < dist_index2wrist_ratio and dist_middle2wrist_ratio < 1.8 and dist_ring2wrist_ratio < 1.6 and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 1
        elif 2.0 < dist_index2wrist_ratio and 1.9 < dist_middle2wrist_ratio and dist_ring2wrist_ratio < 1.6 and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 2
        elif 2.0 < dist_index2wrist_ratio and 1.9 < dist_middle2wrist_ratio and 1.75 < dist_ring2wrist_ratio and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 3
        elif 2.0 < dist_index2wrist_ratio and 1.9 < dist_middle2wrist_ratio and 1.75 < dist_ring2wrist_ratio and 1.5 < dist_pinky2wrist_ratio and dist_pinky_mcp2wrist_ratio < 0.8:
            return 4
        elif dist_index2wrist_ratio > 0.5 and dist_middle2wrist_ratio > 0.5 and dist_ring2wrist_ratio > 0.5 and 0.9 < dist_pinky_mcp2wrist_ratio < 1.2:
            return 5
        elif dist_index2wrist_ratio < 0.5 and dist_middle2wrist_ratio < 0.5 and dist_ring2wrist_ratio < 0.5:
            return 6
        else:
            return -1

    def spin(self):   
        rospy.spin()  

if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass
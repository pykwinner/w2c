#!/usr/bin/env python3

import cv2
import sys
import time
import rospy

rospy.init_node('yolo_detect_node')

from upros_yolo.rknnpool import rknnPoolExecutor
from upros_yolo.func import myFunc

cap = cv2.VideoCapture('http://0.0.0.0:8080/stream?topic=/camera/color/image_raw')

current_path = sys.path[0]
modelPath = current_path + "/rknnModel/yolov5s_relu_tk2_RK3588_i8.rknn"

# 线程数, 增大可提高帧率
TPEs = 4

# 初始化rknn池
pool = rknnPoolExecutor(
    rknnModel=modelPath,
    TPEs=TPEs,
    func=myFunc)

# 初始化异步所需要的帧
if (cap.isOpened()):
    for i in range(TPEs + 1):
        ret, frame = cap.read()
        if not ret:
            cap.release()
            del pool
            exit(-1)
        pool.put(frame)

frames, loopTime, initTime = 0, time.time(), time.time()
while (not rospy.is_shutdown()):
    frames += 1
    ret, frame = cap.read()
    if not ret:
        break
    pool.put(frame)
    (frame, center), flag = pool.get()
    if flag == False:
        break
    cv2.imshow('test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if frames % 30 == 0:
        print("30帧平均帧率:\t", 30 / (time.time() - loopTime), "帧")
        loopTime = time.time()

print("总平均帧率\t", frames / (time.time() - initTime))

# 释放cap和rknn线程池
cap.release()
cv2.destroyAllWindows()
pool.release()

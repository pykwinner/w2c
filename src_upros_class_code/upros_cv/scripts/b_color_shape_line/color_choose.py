#!/usr/bin/env python3

import rospy
import cv2
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import tkinter as tk
from tkinter import messagebox
from threading import Lock

# 全局变量及锁
current_bgr = None
current_hsv = None
bgr_lock = Lock()
hsv_lock = Lock()
bridge = CvBridge()

def image_callback(msg):
    """ROS图像回调函数，处理图像转换和存储"""
    global current_bgr, current_hsv
    try:
        # 将ROS图像转换为OpenCV格式（BGR）
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 转换为HSV颜色空间
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 使用锁更新全局变量
        with hsv_lock:
            current_hsv = hsv_image
        with bgr_lock:
            current_bgr = cv_image
    except Exception as e:
        rospy.logerr(f"图像处理错误: {str(e)}")

def mouse_callback(event, x, y, flags, param):
    """OpenCV鼠标回调函数，处理点击事件"""
    if event == cv2.EVENT_LBUTTONDOWN:
        with hsv_lock:
            if current_hsv is not None:
                # 检查坐标有效性
                if 0 <= y < current_hsv.shape[0] and 0 <= x < current_hsv.shape[1]:
                    hsv_value = current_hsv[y, x]
                    # 显示HSV值弹窗（注意OpenCV的H范围是0-180）
                    messagebox.showinfo(
                        "HSV Values",
                        f"H (0-180): {hsv_value[0]}\nS (0-255): {hsv_value[1]}\nV (0-255): {hsv_value[2]}"
                    )
                else:
                    messagebox.showerror("错误", "点击位置超出图像范围")
            else:
                messagebox.showerror("错误", "尚未接收到图像数据")

def main():
    # 初始化Tkinter（用于弹窗）
    root = tk.Tk()
    root.withdraw()
    
    # 初始化ROS节点
    rospy.init_node('image_hsv_viewer', anonymous=True)
    
    # 订阅图像话题
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    # 创建OpenCV窗口
    cv2.namedWindow("Camera Viewer")
    cv2.setMouseCallback("Camera Viewer", mouse_callback)
    
    # 启动ROS后台线程
    spin_thread = threading.Thread(target=rospy.spin)
    spin_thread.start()
    
    try:
        # 主显示循环
        while not rospy.is_shutdown():
            # 使用锁获取当前BGR图像
            with bgr_lock:
                frame = current_bgr if current_bgr is not None else None
            
            # 更新显示窗口
            if frame is not None:
                cv2.imshow("Camera Viewer", frame)
            
            # 检查退出条件（ESC键或窗口关闭）
            if cv2.waitKey(1) == 27 or cv2.getWindowProperty("Camera Viewer", cv2.WND_PROP_VISIBLE) < 1:
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        cv2.destroyAllWindows()
        rospy.signal_shutdown("程序终止")
        spin_thread.join()
        root.destroy()

if __name__ == '__main__':
    main()
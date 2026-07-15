import cv2  
import numpy as np  
  
# 读取图片  
image = cv2.imread('color_shape.jpeg')  
  
# 将图片从BGR颜色空间转换为HSV颜色空间  
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  
  
# 定义红色在HSV中的范围  
lower_red_1 = np.array([0, 50, 50])  # HSV中的红色下限 (0, 50, 50)  
upper_red_1 = np.array([10, 255, 255])  # HSV中的红色上限 (10, 255, 255)  
  
lower_red_2 = np.array([170, 50, 50])  # HSV中的红色下限 (170, 50, 50)  
upper_red_2 = np.array([180, 255, 255])  # HSV中的红色上限 (180, 255, 255)  
  
# 创建红色掩膜  
mask_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)  
mask_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)  
mask = mask_1 | mask_2  # 合并两个掩膜

cv2.imshow('Mask', mask)  
  
# 对原始图片和掩膜进行位运算，提取红色区域  
red_image = cv2.bitwise_and(image, image, mask=mask)  
  
# 显示原始图片和提取的红色区域  
cv2.imshow('Original Image', image)  
cv2.imshow('Red Extracted', red_image)  
  
# 等待用户按下任意键退出  
cv2.waitKey(0)  
  
# 关闭所有窗口  
cv2.destroyAllWindows()
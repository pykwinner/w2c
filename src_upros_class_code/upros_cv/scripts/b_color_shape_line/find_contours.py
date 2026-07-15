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

# 寻找连通域  
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
  
# 遍历每一个连通域  
for contour in contours:  
    # 获取连通域的边界框  
    x, y, w, h = cv2.boundingRect(contour)  
      
    # 在原始图像上画出连通域的边界框 
    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  
      
    # 计算边界框的中心点坐标  
    center_x = x + w // 2  
    center_y = y + h // 2   
    
    # 在边界框中心画十字  
    # 画水平线  
    cv2.line(image, (center_x - 5, center_y), (center_x + 5, center_y), (0, 255, 0), 2)  
    # 画垂直线  
    cv2.line(image, (center_x, center_y - 5), (center_x, center_y + 5), (0, 255, 0), 2)

      
  
# 显示标有连通域边界框的原始图像  
cv2.imshow('Original Image with Bounding Boxes', image)   
  
# 等待用户按下任意键退出  
cv2.waitKey(0)  
  
# 关闭所有窗口  
cv2.destroyAllWindows()
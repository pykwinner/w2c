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

# 对Mask应用Canny边缘检测  
edges = cv2.Canny(mask, 100, 200)   
  
# 定义一个结构元素，这里使用3x3的矩形作为示例  
kernel = np.ones((3, 3), np.uint8)  
   
# 对掩膜进行膨胀操作  
dilation_edges = cv2.dilate(edges, kernel, iterations=1)

# 使用霍夫变换在边缘检测后的图像中检测圆形  
circles = cv2.HoughCircles(dilation_edges, cv2.HOUGH_GRADIENT, 1, 10, param1=50, param2=40, minRadius=0, maxRadius=0)  
  
# 将检测到的圆形转换为整数  
if circles is not None:  
    circles = np.uint16(np.around(circles))  
  
# 在原始图像上绘制检测到的圆形  
for circle in circles[0, :]:  
    # 绘制圆形  
    cv2.circle(image, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)  
    # 绘制圆心  
    cv2.circle(image, (circle[0], circle[1]), 2, (255, 0, 0), 3)  
  
# 显示原始图片和提取的红色区域  
cv2.imshow('Original Image', image)  

  
# 等待用户按下任意键退出  
cv2.waitKey(0)  
  
# 关闭所有窗口  
cv2.destroyAllWindows()
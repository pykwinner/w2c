import numpy as np
import cv2


class PointerDialReader:
    def __init__(self):
        # 表盘参数
        self.center = None
        self.radius = 0
        self.pointer_angle = 0

    def find_dial_center(self, dial_image):
        """
        使用霍夫圆检测找到表盘中心
        """
        if dial_image is not None:
            gray = cv2.cvtColor(dial_image, cv2.COLOR_BGR2GRAY)       
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            # find circle by hough
            circles = cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,dp=1,minDist=20,param1=50,param2=30,minRadius=10,maxRadius=0)
            # get center of watch
            if circles is not None:
                circles = np.uint16(np.around(circles))  # 取整
                x, y, r = circles[0][0]
                self.center = (x, y)
                self.radius = r
            return self.center, self.radius

    def detect_pointer(self, dial_image, center, radius):
        """
        检测单指针
        """
        # 转换为灰度并增强对比度
        gray = cv2.cvtColor(dial_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)

        # 自适应直方图均衡化（消除阴影）
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)

        # 自适应阈值处理
        thresh = cv2.adaptiveThreshold(enhanced, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV, 11, 2)

        # 形态学操作（针对性处理阴影）
        kernel = np.ones((3,3), np.uint8)
        processed = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        processed = cv2.morphologyEx(processed, cv2.MORPH_CLOSE, kernel)

        # 边缘检测优化（降低阴影边缘干扰）
        edges = cv2.Canny(processed, 70, 150, L2gradient=True)

        # ROI 掩码（强制限定检测区域在表盘有效范围内）
        if center is not None:
            mask = np.zeros_like(edges)
            cv2.circle(mask, center, int(radius * 1.1), 255, -1)
            edges = cv2.bitwise_and(edges, edges, mask=mask)

        # cv2.imshow("Shadow Processed", np.hstack([
        #     gray, enhanced, thresh, processed, edges
        # ]))

        # 使用霍夫直线检测
        lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=60,minLineLength=self.radius*0.5,maxLineGap=10)

        # 后处理
        if lines is not None:
            # 1. 过滤掉不经过表盘中心附近的直线
            filtered_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 计算直线与表盘中心的距离
                distance_to_center = min(
                    np.sqrt((x1 - center[0]) ** 2 + (y1 - center[1]) ** 2),
                    np.sqrt((x2 - center[0]) ** 2 + (y2 - center[1]) ** 2)
                )
                # 如果距离小于半径的某个比例，则保留该直线
                if distance_to_center < radius * 0.5:  # 可以根据实际情况调整这个比例
                    filtered_lines.append(line)
            lines = np.array(filtered_lines)
            # print(f"filtered_lines: {filtered_lines}")

            # 2. 找到最长的直线（指针）
            if len(lines) > 0:
                pointer = max(lines, key=lambda x: np.linalg.norm(x[0][2:] - x[0][:2]))[0]
                return pointer

        return None

    def calculate_angle(self, line, center):
        """
        计算指针相对于垂直线的角度
        """

        x1, y1, x2, y2 = line
        cx, cy = center

        # 确定指针的头部（距离中心较远的端点）
        d1 = np.sqrt((x1 - cx)**2 + (y1 - cy)**2)
        d2 = np.sqrt((x2 - cx)**2 + (y2 - cy)**2)

        # 如果第一个点离中心更远，则它是头部，否则交换两点
        if d1 > d2:
            pointer_tip = (x1, y1)
            pointer_tail = (x2, y2)
        else:
            pointer_tip = (x2, y2)
            pointer_tail = (x1, y1)

        # 计算指针方向向量（从尾部指向头部）
        vx = pointer_tip[0] - pointer_tail[0]
        vy = pointer_tip[1] - pointer_tail[1]

        # 计算垂直向上的向量（12点方向）
        up_vec = np.array([0, -1])

        # 计算指针向量并归一化
        pointer_vec = np.array([vx, vy])
        pointer_vec = pointer_vec / np.linalg.norm(pointer_vec)

        # 计算点积
        dot = np.dot(up_vec, pointer_vec)

        # 计算叉积符号
        cross = np.cross(up_vec, pointer_vec)

        # 计算角度（0-360度）
        angle = np.arccos(dot) * 180 / np.pi
        if cross < 0:
            angle = 360 - angle

        return angle


if __name__ == "__main__":
    dial_reader = PointerDialReader()
    example_line = (1, 1, 0, 0)
    center = (0, 0)
    angle = dial_reader.calculate_angle(example_line, center)
    print(angle)

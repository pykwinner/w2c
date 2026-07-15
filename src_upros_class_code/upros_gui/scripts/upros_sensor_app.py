import sys
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Int16MultiArray
from math import atan2, sin, cos
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel

class OdomGUI(QWidget):
    
    update_odom_signal = pyqtSignal(float, float, float)  
    update_tof_1_signal = pyqtSignal(float)
    update_tof_2_signal = pyqtSignal(float)
    update_tof_3_signal = pyqtSignal(float)
    update_tof_4_signal = pyqtSignal(float)
    update_sonar_1_signal = pyqtSignal(float)
    update_sonar_2_signal = pyqtSignal(float)
    update_sonar_3_signal = pyqtSignal(float)
    update_sonar_4_signal = pyqtSignal(float)
    update_bump_signal = pyqtSignal(int, int, int, int)
    update_servo_signal = pyqtSignal(int, int, int, int, int)

    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_ros()
        self.update_odom_signal.connect(self.update_odom_labels)  # 连接信号与槽
        
        self.update_tof_1_signal.connect(self.update_tof_1_labels)
        self.update_tof_2_signal.connect(self.update_tof_2_labels)
        self.update_tof_3_signal.connect(self.update_tof_3_labels)
        self.update_tof_4_signal.connect(self.update_tof_4_labels)
        
        self.update_sonar_1_signal.connect(self.update_sonar_1_labels)
        self.update_sonar_2_signal.connect(self.update_sonar_2_labels)
        self.update_sonar_3_signal.connect(self.update_sonar_3_labels)
        self.update_sonar_4_signal.connect(self.update_sonar_4_labels)
        
        self.update_bump_signal.connect(self.update_bump_labels)
        self.update_servo_signal.connect(self.update_servo_labels)

    def init_ui(self):
        self.setWindowTitle("UPRos Base Monitor")
        self.setGeometry(300, 300, 400, 200)
        
        # 布局与控件
        layout = QVBoxLayout()
        self.label_x = QLabel("里程计 X: 0.00 m", self)
        self.label_y = QLabel("里程计 Y: 0.00 m", self)
        self.label_yaw = QLabel("里程计 Yaw: 0.00°", self)
        
        self.label_tof_1 = QLabel("TOF 1 距离: 0.00 m", self)
        self.label_tof_2 = QLabel("TOF 2 距离: 0.00 m", self)
        self.label_tof_3 = QLabel("TOF 3 距离: 0.00 m", self)
        self.label_tof_4 = QLabel("TOF 4 距离: 0.00 m", self)

        self.label_sonar_1 = QLabel("超声 1 距离: 0.00 m", self)
        self.label_sonar_2 = QLabel("超声 2 距离: 0.00 m", self)
        self.label_sonar_3 = QLabel("超声 3 距离: 0.00 m", self)
        self.label_sonar_4 = QLabel("超声 4 距离: 0.00 m", self)
        
        self.label_bump_1 =  QLabel("碰撞 1 未触发", self)
        self.label_bump_2 =  QLabel("碰撞 2 未触发", self)
        self.label_bump_3 =  QLabel("碰撞 3 未触发", self)
        self.label_bump_4 =  QLabel("碰撞 4 未触发", self)

        self.label_servo_pose_1 =  QLabel("舵机 1 位置 0 ", self)
        self.label_servo_pose_2 =  QLabel("舵机 2 位置 0 ", self)
        self.label_servo_pose_3 =  QLabel("舵机 3 位置 0 ", self)
        self.label_servo_pose_4 =  QLabel("舵机 4 位置 0 ", self)
        self.label_servo_pose_5 =  QLabel("舵机 5 位置 0 ", self)

        # 设置字体样式（参考网页5的CSS样式）
        for label in [self.label_x, self.label_y, self.label_yaw]:
            label.setStyleSheet("font-size: 20px; color: #2c3e50;")
            layout.addWidget(label)

        # 设置字体样式（参考网页5的CSS样式）
        for label in [self.label_tof_1, self.label_tof_2, self.label_tof_3, self.label_tof_4]:
            label.setStyleSheet("font-size: 20px; color: #2c3e50;")
            layout.addWidget(label)

        # 设置字体样式（参考网页5的CSS样式）
        for label in [self.label_sonar_1, self.label_sonar_2, self.label_sonar_3, self.label_sonar_4]:
            label.setStyleSheet("font-size: 20px; color: #2c3e50;")
            layout.addWidget(label)

        # 设置字体样式（参考网页5的CSS样式）
        for label in [self.label_bump_1, self.label_bump_2, self.label_bump_3, self.label_bump_4]:
            label.setStyleSheet("font-size: 20px; color: #2c3e50;")
            layout.addWidget(label)

        # 设置字体样式（参考网页5的CSS样式）
        for label in [self.label_servo_pose_1, self.label_servo_pose_2, self.label_servo_pose_3, self.label_servo_pose_4, self.label_servo_pose_5]:
            label.setStyleSheet("font-size: 20px; color: #2c3e50;")
            layout.addWidget(label)
                   
        self.setLayout(layout)

    def init_ros(self):
        rospy.init_node('odom_gui_node', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/us/tof1", Range, self.us_tof_1_callback)
        rospy.Subscriber("/us/tof2", Range, self.us_tof_2_callback)
        rospy.Subscriber("/us/tof3", Range, self.us_tof_3_callback)
        rospy.Subscriber("/us/tof4", Range, self.us_tof_4_callback)
        rospy.Subscriber("/ul/sensor1", Range, self.ul_sensor_1_callback)
        rospy.Subscriber("/ul/sensor2", Range, self.ul_sensor_2_callback)
        rospy.Subscriber("/ul/sensor3", Range, self.ul_sensor_3_callback)
        rospy.Subscriber("/ul/sensor4", Range, self.ul_sensor_4_callback)
        rospy.Subscriber("/robot/bump_sensor", Int16MultiArray, self.bump_callback)
        rospy.Subscriber("/robot/servo_position", Int16MultiArray, self.servo_pose_callback)
        
    def odom_callback(self, msg):
        # 解析四元数转欧拉角
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        self.update_odom_signal.emit(x, y, yaw)  # 发射信号
    
    def ul_sensor_1_callback(self, msg):
        range = msg.range
        self.update_sonar_1_signal.emit(range)  # 发射信号

    def ul_sensor_2_callback(self, msg):
        range = msg.range
        self.update_sonar_2_signal.emit(range)  # 发射信号

    def ul_sensor_3_callback(self, msg):
        range = msg.range
        self.update_sonar_3_signal.emit(range)  # 发射信号

    def ul_sensor_4_callback(self, msg):
        range = msg.range
        self.update_sonar_4_signal.emit(range)  # 发射信号

    def us_tof_1_callback(self, msg):
        range = msg.range
        self.update_tof_1_signal.emit(range)  # 发射信号
        
    def us_tof_2_callback(self, msg):
        range = msg.range
        self.update_tof_2_signal.emit(range)  # 发射信号
        
    def us_tof_3_callback(self, msg):
        range = msg.range
        self.update_tof_3_signal.emit(range)  # 发射信号

    def us_tof_4_callback(self, msg):
        range = msg.range
        self.update_tof_4_signal.emit(range)  # 发射信号
        
    def bump_callback(self,msg):
        bump_1 = msg.data[0]
        bump_2 = msg.data[1]
        bump_3 = msg.data[2]
        bump_4 = msg.data[3]
        self.update_bump_signal.emit(bump_1, bump_2, bump_3, bump_4)

    def servo_pose_callback(self,msg):
        servo_1 = msg.data[0]
        servo_2 = msg.data[1]
        servo_3 = msg.data[2]
        servo_4 = msg.data[3]
        servo_5 = msg.data[4]
        self.update_servo_signal.emit(servo_1, servo_2, servo_3, servo_4, servo_5)

    def update_odom_labels(self, x, y, yaw):
        # 更新界面数据
        self.label_x.setText(f"X: {x:.2f} m")
        self.label_y.setText(f"Y: {y:.2f} m")
        self.label_yaw.setText(f"Yaw: {yaw * 57.3:.2f}°")  # 弧度转角度

    def update_tof_1_labels(self, range):
        # 更新界面数据
        self.label_tof_1.setText(f"TOF 1 距离: {range:.2f} m")

    def update_tof_2_labels(self, range):
        # 更新界面数据
        self.label_tof_2.setText(f"TOF 2 距离: {range:.2f} m")

    def update_tof_3_labels(self, range):
        # 更新界面数据
        self.label_tof_3.setText(f"TOF 3 距离: {range:.2f} m")
        
    def update_tof_4_labels(self, range):
        # 更新界面数据
        self.label_tof_4.setText(f"TOF 4 距离: {range:.2f} m")

    def update_sonar_1_labels(self, range):
        # 更新界面数据
        self.label_sonar_1.setText(f"超声 1 距离: {range:.2f} m")

    def update_sonar_2_labels(self, range):
        # 更新界面数据
        self.label_sonar_2.setText(f"超声 2 距离: {range:.2f} m")

    def update_sonar_3_labels(self, range):
        # 更新界面数据
        self.label_sonar_3.setText(f"超声 3 距离: {range:.2f} m")
        
    def update_sonar_4_labels(self, range):
        # 更新界面数据
        self.label_sonar_4.setText(f"超声 4 距离: {range:.2f} m")
    
    def update_bump_labels(self, bump_1, bump_2, bump_3, bump_4):
        if bump_1 == 0:
            self.label_bump_1.setText(f"碰撞 1 未触发")
        elif bump_1 == 1:
            self.label_bump_1.setText(f"碰撞 1 触发")
        if bump_2 == 0:
            self.label_bump_2.setText(f"碰撞 2 未触发")
        elif bump_2 == 1:
            self.label_bump_2.setText(f"碰撞 2 触发")
        if bump_3 == 0:
            self.label_bump_3.setText(f"碰撞 3 未触发")
        elif bump_3 == 1:
            self.label_bump_3.setText(f"碰撞 3 触发")
        if bump_4 == 0:
            self.label_bump_4.setText(f"碰撞 4 未触发")
        elif bump_4 == 1:
            self.label_bump_4.setText(f"碰撞 4 触发")
    
    def update_servo_labels(self, servo_1, servo_2, servo_3, servo_4, servo_5):
        self.label_servo_pose_1.setText(f"舵机 1 位置: {servo_1:d} ")
        self.label_servo_pose_2.setText(f"舵机 2 位置: {servo_2:d} ")
        self.label_servo_pose_3.setText(f"舵机 3 位置: {servo_3:d} ")
        self.label_servo_pose_4.setText(f"舵机 4 位置: {servo_4:d} ")
        self.label_servo_pose_5.setText(f"舵机 5 位置: {servo_5:d} ")
            
    def ros_spin(self):
        rospy.spin()  # 非阻塞式处理ROS消息

    def closeEvent(self, event):
        rospy.signal_shutdown("GUI closed")
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OdomGUI()
    window.show()
    sys.exit(app.exec_())
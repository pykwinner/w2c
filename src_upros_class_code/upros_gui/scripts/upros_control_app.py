import sys
import rospy
import time
from geometry_msgs.msg import Twist
from upros_message.msg import SingleServo
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QLabel

class RobotControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.speed_factor = 1.0  # 速度倍率默认100%
        self.init_ros()
        self.init_ui()

    def init_ros(self):
        rospy.init_node('pyqt_robot_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.servo_pub = rospy.Publisher('/single_servo_topic', SingleServo, queue_size=10)
        for i in range (5):
            servo = SingleServo()
            servo.ID = i+1
            servo.Target_position_Angle = 0
            servo.Rotation_Speed = 100
            self.servo_pub.publish(servo)  
            time.sleep(0.1)

    def init_ui(self):
        self.setWindowTitle('ROS机器人控制面板')
        layout = QVBoxLayout()

        # 紧急停止按钮（新增功能）
        self.btn_emergency = QPushButton('紧急停止', self)
        self.btn_emergency.setStyleSheet("background-color: red; color: white;")
        layout.addWidget(self.btn_emergency)

        # 速度调节滑块
        hbox_slider = QHBoxLayout()
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)      # 0%倍率[1](@ref)
        self.speed_slider.setMaximum(200)    # 200%倍率[2](@ref)
        self.speed_slider.setSingleStep(10)  # 步长10%
        self.speed_slider.setValue(100)      # 默认100%
        self.speed_slider.setTickInterval(20)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_label = QLabel('速度倍率: 100%', self)
        hbox_slider.addWidget(self.speed_slider)
        hbox_slider.addWidget(self.speed_label)
        layout.addLayout(hbox_slider)

        # 原有控制按钮
        hbox_linear = QHBoxLayout()
        self.btn_forward = QPushButton('前进', self)
        self.btn_backward = QPushButton('后退', self)
        hbox_linear.addWidget(self.btn_forward)
        hbox_linear.addWidget(self.btn_backward)

        hbox_lateral = QHBoxLayout()
        self.btn_left_shift = QPushButton('左移', self)
        self.btn_right_shift = QPushButton('右移', self)
        hbox_lateral.addWidget(self.btn_left_shift)
        hbox_lateral.addWidget(self.btn_right_shift)

        hbox_rotation = QHBoxLayout()
        self.btn_left_turn = QPushButton('左转', self)
        self.btn_right_turn = QPushButton('右转', self)
        hbox_rotation.addWidget(self.btn_left_turn)
        hbox_rotation.addWidget(self.btn_right_turn)

        vbox_servos = QVBoxLayout()
        # 舵机 1 调节滑块
        hbox_servo_1_slider = QHBoxLayout()
        self.servo_1_slider = QSlider(Qt.Horizontal)
        self.servo_1_slider.setMinimum(-500)
        self.servo_1_slider.setMaximum(500)
        self.servo_1_slider.setSingleStep(10)  # 步长10%
        self.servo_1_slider.setValue(0)      # 默认0
        self.servo_1_slider.setTickInterval(20)
        self.servo_1_slider.setTickPosition(QSlider.TicksBelow)
        self.servo_1_label = QLabel('舵机 1 位置: 0', self)
        hbox_servo_1_slider.addWidget(self.servo_1_slider)
        hbox_servo_1_slider.addWidget(self.servo_1_label)
        vbox_servos.addLayout(hbox_servo_1_slider)
        # 舵机 2 调节滑块
        hbox_servo_2_slider = QHBoxLayout()
        self.servo_2_slider = QSlider(Qt.Horizontal)
        self.servo_2_slider.setMinimum(-500)
        self.servo_2_slider.setMaximum(500)
        self.servo_2_slider.setSingleStep(10)  # 步长10%
        self.servo_2_slider.setValue(0)      # 默认0
        self.servo_2_slider.setTickInterval(20)
        self.servo_2_slider.setTickPosition(QSlider.TicksBelow)
        self.servo_2_label = QLabel('舵机 2 位置: 0', self)
        hbox_servo_2_slider.addWidget(self.servo_2_slider)
        hbox_servo_2_slider.addWidget(self.servo_2_label)
        vbox_servos.addLayout(hbox_servo_2_slider)
        # 舵机 3 调节滑块
        hbox_servo_3_slider = QHBoxLayout()
        self.servo_3_slider = QSlider(Qt.Horizontal)
        self.servo_3_slider.setMinimum(-500)
        self.servo_3_slider.setMaximum(500)
        self.servo_3_slider.setSingleStep(10)  # 步长10%
        self.servo_3_slider.setValue(0)      # 默认0
        self.servo_3_slider.setTickInterval(20)
        self.servo_3_slider.setTickPosition(QSlider.TicksBelow)
        self.servo_3_label = QLabel('舵机 3 位置: 0', self)
        hbox_servo_3_slider.addWidget(self.servo_3_slider)
        hbox_servo_3_slider.addWidget(self.servo_3_label)
        vbox_servos.addLayout(hbox_servo_3_slider)
        # 舵机 4 调节滑块
        hbox_servo_4_slider = QHBoxLayout()
        self.servo_4_slider = QSlider(Qt.Horizontal)
        self.servo_4_slider.setMinimum(-500)
        self.servo_4_slider.setMaximum(500)
        self.servo_4_slider.setSingleStep(10)  # 步长10%
        self.servo_4_slider.setValue(0)      # 默认0
        self.servo_4_slider.setTickInterval(20)
        self.servo_4_slider.setTickPosition(QSlider.TicksBelow)
        self.servo_4_label = QLabel('舵机 4 位置: 0', self)
        hbox_servo_4_slider.addWidget(self.servo_4_slider)
        hbox_servo_4_slider.addWidget(self.servo_4_label)
        vbox_servos.addLayout(hbox_servo_4_slider)
        # 舵机 5 调节滑块
        hbox_servo_5_slider = QHBoxLayout()
        self.servo_5_slider = QSlider(Qt.Horizontal)
        self.servo_5_slider.setMinimum(-500)
        self.servo_5_slider.setMaximum(500)
        self.servo_5_slider.setSingleStep(10)  # 步长10%
        self.servo_5_slider.setValue(0)      # 默认0
        self.servo_5_slider.setTickInterval(20)
        self.servo_5_slider.setTickPosition(QSlider.TicksBelow)
        self.servo_5_label = QLabel('舵机 5 位置: 0', self)
        hbox_servo_5_slider.addWidget(self.servo_5_slider)
        hbox_servo_5_slider.addWidget(self.servo_5_label)
        vbox_servos.addLayout(hbox_servo_5_slider)
        # 舵机 6 调节滑块
        hbox_servo_6_slider = QHBoxLayout()
        self.servo_6_slider = QSlider(Qt.Horizontal)
        self.servo_6_slider.setMinimum(-500)
        self.servo_6_slider.setMaximum(500)
        self.servo_6_slider.setSingleStep(10)  # 步长10%
        self.servo_6_slider.setValue(0)      # 默认0
        self.servo_6_slider.setTickInterval(20)
        self.servo_6_slider.setTickPosition(QSlider.TicksBelow)
        self.servo_6_label = QLabel('舵机 6 位置: 0', self)
        hbox_servo_6_slider.addWidget(self.servo_6_slider)
        hbox_servo_6_slider.addWidget(self.servo_6_label)
        vbox_servos.addLayout(hbox_servo_6_slider)

        layout.addLayout(hbox_linear)
        layout.addLayout(hbox_lateral)
        layout.addLayout(hbox_rotation)
        layout.addLayout(vbox_servos)
        self.setLayout(layout)

        # 信号连接（新增功能）
        self.speed_slider.valueChanged.connect(self.update_speed_factor)
        self.btn_emergency.clicked.connect(self.emergency_stop)
        self.btn_forward.clicked.connect(lambda: self.publish_cmd_vel(linear_x=0.1))
        self.btn_backward.clicked.connect(lambda: self.publish_cmd_vel(linear_x=-0.1))
        self.btn_left_shift.clicked.connect(lambda: self.publish_cmd_vel(linear_y=0.1))
        self.btn_right_shift.clicked.connect(lambda: self.publish_cmd_vel(linear_y=-0.1))
        self.btn_left_turn.clicked.connect(lambda: self.publish_cmd_vel(angular_z=0.25))
        self.btn_right_turn.clicked.connect(lambda: self.publish_cmd_vel(angular_z=-0.25))
        self.servo_1_slider.valueChanged.connect(self.publish_servo_position_1)
        self.servo_2_slider.valueChanged.connect(self.publish_servo_position_2)     
        self.servo_3_slider.valueChanged.connect(self.publish_servo_position_3)   
        self.servo_4_slider.valueChanged.connect(self.publish_servo_position_4)
        self.servo_5_slider.valueChanged.connect(self.publish_servo_position_5)
        self.servo_6_slider.valueChanged.connect(self.publish_servo_position_6)


    def update_speed_factor(self, value):
        """更新速度倍率并显示"""
        self.speed_factor = value / 100.0  # 转换为浮点倍率[1](@ref)
        self.speed_label.setText(f'速度倍率: {value}%')

    def publish_servo_position_1(self, value):
        self.servo_1_label.setText(f'舵机 1 位置: {value}')   
        servo = SingleServo()
        servo.ID = 1
        servo.Target_position_Angle = value
        servo.Rotation_Speed = 100
        self.servo_pub.publish(servo)    

    def publish_servo_position_2(self, value):
        self.servo_2_label.setText(f'舵机 2 位置: {value}')   
        servo = SingleServo()
        servo.ID = 2
        servo.Target_position_Angle = value
        servo.Rotation_Speed = 100
        self.servo_pub.publish(servo)  

    def publish_servo_position_3(self, value):
        self.servo_3_label.setText(f'舵机 3 位置: {value}')   
        servo = SingleServo()
        servo.ID = 3
        servo.Target_position_Angle = value
        servo.Rotation_Speed = 100
        self.servo_pub.publish(servo)  

    def publish_servo_position_4(self, value):
        self.servo_4_label.setText(f'舵机 4 位置: {value}')   
        servo = SingleServo()
        servo.ID = 4
        servo.Target_position_Angle = value
        servo.Rotation_Speed = 100
        self.servo_pub.publish(servo)  

    def publish_servo_position_5(self, value):
        self.servo_5_label.setText(f'舵机 5 位置: {value}')   
        servo = SingleServo()
        servo.ID = 5
        servo.Target_position_Angle = value
        servo.Rotation_Speed = 100
        self.servo_pub.publish(servo)  

    def publish_servo_position_6(self, value):
        self.servo_6_label.setText(f'舵机 6 位置: {value}')   
        servo = SingleServo()
        servo.ID = 6
        servo.Target_position_Angle = value
        servo.Rotation_Speed = 100
        self.servo_pub.publish(servo)   

    def publish_cmd_vel(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """发布带速度倍率的指令"""
        twist = Twist()
        twist.linear.x = linear_x * self.speed_factor
        twist.linear.y = linear_y * self.speed_factor
        twist.angular.z = angular_z * self.speed_factor
        self.pub.publish(twist)
        print(f"发送指令：x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, z={twist.angular.z:.2f}")

    def emergency_stop(self):
        """紧急停止功能"""
        self.publish_cmd_vel(0, 0, 0)
        print("紧急停止：所有速度归零")

    def closeEvent(self, event):
        self.emergency_stop()
        rospy.signal_shutdown("GUI closed")
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotControlGUI()
    ex.show()
    sys.exit(app.exec_())
# 导入库文件
import sys,os
if hasattr(sys, 'frozen'):
    os.environ['PATH'] = sys._MEIPASS + ";" + os.environ['PATH']
from PyQt5.QtWidgets import *
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QApplication, QMainWindow
from main_windows import Ui_MainWindow
from robot_communication import RobotCommunication

class MainWindow(QMainWindow, Ui_MainWindow):
    # 初始化
    def __init__(self,parent = None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.robot = RobotCommunication(self)
        # 设置实例
        self.CreateItems()
        # 设置信号和槽
        self.CreateSignalSlot()
        # 设置速度
        self.line_speed = 100
        self.trun_speed = 50
        self.servo_speed = 50
        self.show_img_version()

    def show_img_version(self):
        version_file = "/home/bcsh/.img_version.txt"
        try:
            with open(version_file, 'r') as file:
                version = file.read()
            self.img_version.setText("镜像版本：" + version)
        except:
            pass

    # 设置实例
    def CreateItems(self):
        self.serial_push()

    # 设置信号与槽
    def CreateSignalSlot(self):
        self.btn_scan.clicked.connect(self.serial_push)
        self.btn_open_serial.clicked.connect(self.Com_Open)
        self.btn_close_serial.clicked.connect(self.Com_Close)
        self.btn_after.clicked.connect(self.move_up)
        self.btn_before.clicked.connect(self.move_down)
        self.btn_left.clicked.connect(self.move_left)
        self.btn_right.clicked.connect(self.move_right)
        self.btn_trun_left.clicked.connect(self.trun_left)
        self.btn_trun_right.clicked.connect(self.trun_right)
        self.btn_clear_odom.clicked.connect(self.clear_odom)
        self.btn_stop_move.clicked.connect(self.move_stop)

    # 向前
    def move_up(self):
        self.robot.robot_move(self.line_speed, 0, 0)

    # 向后
    def move_down(self):
        self.robot.robot_move(-self.line_speed, 0, 0)

    # 向左
    def move_left(self):
        self.robot.robot_move(0, self.line_speed, 0)

    # 向右
    def move_right(self):
        self.robot.robot_move(0, -self.line_speed, 0)

    # 左转
    def trun_left(self):
        self.robot.robot_move(0, 0, self.trun_speed)

    # 右转
    def trun_right(self):
        self.robot.robot_move(0, 0, -self.trun_speed)

    # 停止
    def move_stop(self):
        self.robot.robot_move(0, 0, 0)

    # 清空里程计
    def clear_odom(self):
        self.robot.clear_odom()


    # 超声波TOF数据返回
    def ul_tof_callback(self, tof1, ul1, tof2, ul2, tof3, ul3, tof4, ul4):
        self.tof_1.setText(str(tof1))
        self.tof_2.setText(str(tof2))
        self.tof_3.setText(str(tof3))
        self.tof_4.setText(str(tof4))
        self.ul_1.setText(str(ul1))
        self.ul_2.setText(str(ul2))
        self.ul_3.setText(str(ul3))
        self.ul_4.setText(str(ul4))

    def odom_callback(self, odom_x, odom_y, odom_yaw):
        self.odom_x.setText(str(odom_x) + " mm")
        self.odom_y.setText(str(odom_y) + " mm")
        self.odom_yaw.setText(str(odom_yaw / 100) + " rad")

    def collision_callback(self, collision1, collision2, collision3, collision4):
        self.coll_1.setText("触发" if collision1 == 1 else "未触发")
        self.coll_2.setText("触发" if collision2 == 1 else "未触发")
        self.coll_3.setText("触发" if collision3 == 1 else "未触发")
        self.coll_4.setText("触发" if collision4 == 1 else "未触发")

    # 串口刷新
    def serial_push(self):
        self.cm_port.clear()
        com = QSerialPort()
        com_list = QSerialPortInfo.availablePorts()
        for info in com_list:
            com.setPort(info)
            if com.open(QSerialPort.ReadWrite):
                self.cm_port.addItem(info.portName())
                com.close()

    # 串口打开按钮按下
    def Com_Open(self):
        comName = self.cm_port.currentText()
        if self.robot.open_com(comName, 1000000):
            self.robot.set_ul_tof_callback(self.ul_tof_callback)
            self.robot.set_odom_callback(self.odom_callback)
            self.robot.set_collision_callback(self.collision_callback)

            self.btn_after.setEnabled(True)
            self.btn_before.setEnabled(True)
            self.btn_left.setEnabled(True)
            self.btn_right.setEnabled(True)
            self.btn_trun_right.setEnabled(True)
            self.btn_trun_left.setEnabled(True)
            self.cm_port.setEnabled(False)
            self.btn_clear_odom.setEnabled(True)
            self.btn_stop_move.setEnabled(True)
            self.btn_close_serial.setEnabled(True)
            self.btn_open_serial.setEnabled(False)
            self.robot.set_chassiss_type(1)
        else:
            QMessageBox.critical(self, 'ERROR', '串口打开失败')

    # 串口关闭按钮按下
    def Com_Close(self):
        self.robot.close()
        # 让控件处于不可选状态
        self.btn_after.setEnabled(False)
        self.btn_before.setEnabled(False)
        self.btn_left.setEnabled(False)
        self.btn_right.setEnabled(False)
        self.btn_trun_right.setEnabled(False)
        self.btn_trun_left.setEnabled(False)
        self.cm_port.setEnabled(True)
        self.btn_clear_odom.setEnabled(False)
        self.btn_stop_move.setEnabled(False)
        self.btn_close_serial.setEnabled(False)
        self.btn_open_serial.setEnabled(True)

def main():
    window = QApplication(sys.argv)
    TheWin = MainWindow()
    TheWin.show()
    sys.exit(window.exec_())

if __name__ == '__main__':
    main()


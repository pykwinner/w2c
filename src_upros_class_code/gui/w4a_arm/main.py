# 导入库文件
import sys, os
import data_frame

if hasattr(sys, 'frozen'):
    os.environ['PATH'] = sys._MEIPASS + ";" + os.environ['PATH']
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QApplication, QMainWindow
from main_windows import Ui_MainWindow
from robot_communication import RobotCommunication

class MainWindow(QMainWindow, Ui_MainWindow):
    # 初始化
    def __init__(self,parent = None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.arm = RobotCommunication(self)
        # 设置实例
        self.CreateItems()
        # 设置信号和槽
        self.CreateSignalSlot()
        self.step = 200

    # 设置实例
    def CreateItems(self):
        self.serial_push()

    # 设置信号与槽
    def CreateSignalSlot(self):
        self.btn_scan.clicked.connect(self.serial_push)
        self.btn_open_serial.clicked.connect(self.Com_Open)
        self.btn_close_serial.clicked.connect(self.Com_Close)
        self.btn_pick.clicked.connect(self.arm_pick)
        self.btn_up.clicked.connect(self.arm_up)
        self.btn_step_x_in.clicked.connect(self.step_x_in)
        self.btn_step_x_re.clicked.connect(self.step_x_re)
        self.btn_step_y_in.clicked.connect(self.step_y_in)
        self.btn_step_y_re.clicked.connect(self.step_y_re)
        self.btn_step_z_in.clicked.connect(self.step_z_in)
        self.btn_step_z_re.clicked.connect(self.step_z_re)
        self.btn_arm_reset.clicked.connect(self.arm_reset)
        self.btn_set_pos.clicked.connect(self.set_pos_control)

        self.btn_step_x_in.setEnabled(False)
        self.btn_step_x_re.setEnabled(False)
        self.btn_step_y_in.setEnabled(False)
        self.btn_step_y_re.setEnabled(False)
        self.btn_step_z_in.setEnabled(False)
        self.btn_step_z_re.setEnabled(False)
        self.btn_pick.setEnabled(False)
        self.btn_up.setEnabled(False)
        self.btn_set_pos.setEnabled(False)
        self.btn_arm_reset.setEnabled(False)

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
        self.cm_port.setEnabled(False)
        self.btn_open_serial.setEnabled(False)
        self.btn_close_serial.setEnabled(True)
        self.arm.open_com(comName, 115200)
        self.arm.set_armStatus_callback(self.get_arm_status)

        self.btn_step_x_in.setEnabled(True)
        self.btn_step_x_re.setEnabled(True)
        self.btn_step_y_in.setEnabled(True)
        self.btn_step_y_re.setEnabled(True)
        self.btn_step_z_in.setEnabled(True)
        self.btn_step_z_re.setEnabled(True)
        self.btn_pick.setEnabled(True)
        self.btn_up.setEnabled(True)
        self.btn_set_pos.setEnabled(True)
        self.btn_arm_reset.setEnabled(True)

    # 串口关闭按钮按下
    def Com_Close(self):
        # 让控件处于不可选状态
        self.arm.close()
        self.cm_port.setEnabled(True)
        self.btn_open_serial.setEnabled(True)
        self.btn_close_serial.setEnabled(False)

        self.btn_step_x_in.setEnabled(False)
        self.btn_step_x_re.setEnabled(False)
        self.btn_step_y_in.setEnabled(False)
        self.btn_step_y_re.setEnabled(False)
        self.btn_step_z_in.setEnabled(False)
        self.btn_step_z_re.setEnabled(False)
        self.btn_pick.setEnabled(False)
        self.btn_up.setEnabled(False)
        self.btn_set_pos.setEnabled(False)
        self.btn_arm_reset.setEnabled(False)

    def get_arm_status(self, data_size, data):
        if data[1] == data_frame.GET_ARM_STATUS and data_size == 24:
            arm_state = int(data[3])
            key_state = int(data[5])
            pump_valve_state = bin(int(data[6]))[2:].zfill(8)
            pump_state = pump_valve_state[7]
            valve_state = pump_valve_state[6]

            pos_x = int.from_bytes(data[11:15], byteorder='little', signed=True)
            pos_y = int.from_bytes(data[15:19], byteorder='little', signed=True)
            pos_z = int.from_bytes(data[19:23], byteorder='little', signed=True)

            self.set_arm_state(arm_state)
            self.set_key_state(key_state)
            self.set_pump_valve_state(pump_state, valve_state)

            self.lb_pos_x.setText(str(pos_x))
            self.lb_pos_y.setText(str(pos_y))
            self.lb_pos_z.setText(str(pos_z))

    # 机械臂抓取物体
    def arm_pick(self):
        self.arm.arm_pick()

    # 机械臂放置物体
    def arm_up(self):
        self.arm.arm_up()

    def step_x_in(self):
        self.arm.set_step_x(self.step)

    def step_x_re(self):
        self.arm.set_step_x(-self.step)

    def step_y_in(self):
        self.arm.set_step_y(self.step)

    def step_y_re(self):
        self.arm.set_step_y(-self.step)

    def step_z_in(self):
        self.arm.set_step_z(self.step)

    def step_z_re(self):
        self.arm.set_step_z(-self.step)

    # 机械臂重置校准
    def arm_reset(self):
        self.arm.reset_arm()

    def set_pos_control(self):
        x = int(self.ed_pos_x.text())
        y = int(self.ed_pos_y.text())
        z = int(self.ed_pos_z.text())
        self.arm.set_pos_x(x)
        self.arm.set_pos_y(y)
        self.arm.set_pos_z(z)
    # 设置机械臂状态
    def set_arm_state(self, state):
        if state == 0:
            self.arm_run_state.setText("正常")
        if 0 < state < 6:
            self.arm_run_state.setText("重置校准中")
        if state == 241:
            self.arm_run_state.setText("限位触发")

    # 设置红外状态
    def set_key_state(self, state):
        s = bin(state)[2:].zfill(8)
        if s[7] == '0':
            self.hw_state.setText("未触发")
        else:
            self.hw_state.setText("触发")

    # 设置气泵和电磁阀状态
    def set_pump_valve_state(self, pump_state, valve_state):
        if pump_state == '0':
            self.pump_state.setText("关闭")
        else:
            self.pump_state.setText("打开")

        if valve_state == '0':
            self.valve_state.setText("关闭")
        else:
            self.valve_state.setText("打开")

def main():
    window = QApplication(sys.argv)
    TheWin = MainWindow()
    TheWin.show()
    sys.exit(window.exec_())

if __name__ == '__main__':
    main()
from PyQt5.QtSerialPort import QSerialPort
import data_frame
import time
from PyQt5.QtCore import QTimer

class RobotCommunication:

    def __init__(self, qt):
        self.com = QSerialPort()
        self.com.readyRead.connect(self.com_receive_data)
        self.msg_list = []
        # 开启定时发送消息定时器线程
        self.send_timer = QTimer(qt)
        self.send_timer.timeout.connect(self.send_timer_run)
        # 开启定时发送消息获取机械臂状态线程
        self.get_data_timer = QTimer(qt)
        self.get_data_timer.timeout.connect(self.get_data_run)

    # 打开串口
    def open_com(self, com_port, com_baud):
        self.com.setPortName(com_port)
        self.com.setBaudRate(com_baud)
        try:
            if self.com.open(QSerialPort.ReadWrite) == False:
                print("串口打开失败")
                return
            self.send_timer.start(30)
            self.get_data_timer.start(600)
        except:
            print("串口打开失败")
            return

    # 关闭串口
    def close(self):
        self.com.close()

    # 串口数据接收
    def com_receive_data(self):
        try:
            rxData = bytes(self.com.readAll())
            data_size = len(rxData)
            data = list(rxData)
            self.arm_callback(data_size, data)
        except:
            print("串口数据错误")

    # 定时发送消息列表
    def send_timer_run(self):
        if self.com.isOpen():
            if len(self.msg_list) > 0 :
                self.com.write(self.msg_list[0])
                time.sleep(0.01)
                self.msg_list.remove(self.msg_list[0])

    def set_armStatus_callback(self, arm_status_callback):
        self.arm_callback = arm_status_callback

    # 定时发送消息获取机械臂状态线程
    def get_data_run(self):
        data = [0] * 0
        data_pack = self.generateCmd(data_frame.GET_ARM_STATUS, data)
        self.msg_list.append(data_pack)

    # 生成串口发送数据包
    def generateCmd(self, cmd, data):
        buffer = [0] * (len(data) + 4)
        buffer[0] = data_frame.HEAD
        buffer[1] = cmd
        buffer[2] = len(data)
        for i in range(len(data)):
            buffer[3 + i] = data[i]
        # 校验位
        check = 0
        for i in range(len(buffer)):
            check += buffer[i]
        buffer[len(data) + 3] = check & 0xFF
        # for i in range(len(buffer)):
        #     print(hex(int(buffer[i])))
        return bytes(buffer)

    # 获取机械臂状态
    def get_arm_status(self):
        data = [0] * 0
        data_pack = self.generateCmd(data_frame.GET_ARM_STATUS,data)
        self.msg_list.append(data_pack)

    # 机械臂复位校准
    def reset_arm(self):
        data = [0] * 0
        data_pack = self.generateCmd(data_frame.RESET, data)
        self.msg_list.append(data_pack)

    # 机械臂停止运动
    def stop_arm(self):
        data = [0] * 0
        data_pack = self.generateCmd(data_frame.STOP, data)
        self.msg_list.append(data_pack)

    # 机械臂抓取
    def arm_pick(self):
        # 打开气泵
        data = [0] * 1
        data[0] = 1
        data_pack = self.generateCmd(data_frame.SET_PUMP, data)
        self.msg_list.append(data_pack)
        # 关闭电磁阀
        data[0] = 0
        data_pack = self.generateCmd(data_frame.SET_VALE, data)
        self.msg_list.append(data_pack)

    # 机械臂放置
    def arm_up(self):
        # 关闭气泵
        data = [0] * 1
        data[0] = 0
        data_pack = self.generateCmd(data_frame.SET_PUMP, data)
        self.msg_list.append(data_pack)
        # 打开电磁阀
        data[0] = 1
        data_pack = self.generateCmd(data_frame.SET_VALE, data)
        self.msg_list.append(data_pack)

    def set_step_x(self, step):
        data = step.to_bytes(4, byteorder='little', signed=True)
        data_pack = self.generateCmd(data_frame.SETP_X, data)
        self.msg_list.append(data_pack)

    def set_step_y(self, step):
        data = step.to_bytes(4, byteorder='little', signed=True)
        data_pack = self.generateCmd(data_frame.SETP_Y, data)
        self.msg_list.append(data_pack)

    def set_step_z(self, step):
        data = step.to_bytes(4, byteorder='little', signed=True)
        data_pack = self.generateCmd(data_frame.SETP_Z, data)
        self.msg_list.append(data_pack)

    def set_pos_x(self, step):
        data = step.to_bytes(4, byteorder='little', signed=True)
        data_pack = self.generateCmd(data_frame.POS_X, data)
        self.msg_list.append(data_pack)

    def set_pos_y(self, step):
        data = step.to_bytes(4, byteorder='little', signed=True)
        data_pack = self.generateCmd(data_frame.POS_Y, data)
        self.msg_list.append(data_pack)

    def set_pos_z(self, step):
        data = step.to_bytes(4, byteorder='little', signed=True)
        data_pack = self.generateCmd(data_frame.POS_Z, data)
        self.msg_list.append(data_pack)

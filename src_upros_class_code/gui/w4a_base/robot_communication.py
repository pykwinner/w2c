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
        # 开启定时发送消息获取机器人状态线程
        self.get_data_timer = QTimer(qt)
        self.get_data_timer.timeout.connect(self.get_data_run)

    # 打开串口
    def open_com(self, com_port, com_baud):
        self.com.setPortName(com_port)
        self.com.setBaudRate(com_baud)
        try:
            if self.com.open(QSerialPort.ReadWrite) == False:
                print("串口打开失败")
                return False
            self.send_timer.start(30)
            self.get_data_timer.start(600)
            return True
        except:
            print("串口打开失败")
            return False

    # 关闭串口
    def close(self):
        self.com.close()

    # 串口数据接收
    def com_receive_data(self):
        try:
            rxData = bytes(self.com.readAll())
            data_size = len(rxData)
            data = list(rxData)
            self.parse_data(data_size, data)
        except:
            print("串口数据错误")

    # 定时发送消息列表
    def send_timer_run(self):
        if self.com.isOpen():
            if len(self.msg_list) > 0 :
                self.com.write(self.msg_list[0])
                time.sleep(0.01)
                self.msg_list.remove(self.msg_list[0])


    def parse_data(self, data_size, data):
        if data[0] == data_frame.HEAD1 and data[1] == data_frame.HEAD2:
            # 超声波数据返回
            if data[2] == data_frame.ID_GET_UL_TOF and data_size == 21:
                tof1 = int.from_bytes(data[4:6], byteorder='little', signed=False)
                ul1 = int.from_bytes(data[6:8], byteorder='little', signed=False)
                tof2 = int.from_bytes(data[8:10], byteorder='little', signed=False)
                ul2 = int.from_bytes(data[10:12], byteorder='little', signed=False)
                tof3 = int.from_bytes(data[12:14], byteorder='little', signed=False)
                ul3 = int.from_bytes(data[14:16], byteorder='little', signed=False)
                tof4 = int.from_bytes(data[16:18], byteorder='little', signed=False)
                ul4 = int.from_bytes(data[18:20], byteorder='little', signed=False)
                if self.ul_tof_callback is not None:
                    self.ul_tof_callback(tof1, ul1, tof2, ul2, tof3, ul3, tof4, ul4)

            # 里程计数据返回
            if data[2] == data_frame.ID_GET_ODOM and data_size == 21:
                odom_x = int.from_bytes(data[10:14], byteorder='little', signed=True)
                odom_y = int.from_bytes(data[14:18], byteorder='little', signed=True)
                odom_yaw = int.from_bytes(data[18:20], byteorder='little', signed=True)
                if self.odom_callback is not None:
                    self.odom_callback(odom_x, odom_y, odom_yaw)
            # 传感器数据返回
            if data[2] == data_frame.ID_GET_SENSOR and data_size == 30:
                collision1 = (data[28] >> 0) & 1
                collision2 = (data[28] >> 1) & 1
                collision3 = (data[28] >> 2) & 1
                collision4 = (data[28] >> 3) & 1
                if self.collision_callback is not None:
                    self.collision_callback(collision1, collision2, collision3, collision4)

    # 设置里程计数据回调
    def set_odom_callback(self, data_callback):
        self.odom_callback = data_callback

    # 设置超声TOF数据回调
    def set_ul_tof_callback(self, data_callback):
        self.ul_tof_callback = data_callback

    # 设置碰撞数据回调
    def set_collision_callback(self, data_callback):
        self.collision_callback = data_callback

    # 定时发送消息获取机器人信息
    def get_data_run(self):
        self.get_odom()
        time.sleep(0.1)
        self.get_ul_tof()
        time.sleep(0.1)
        self.get_sensor()

    # 生成串口发送数据包
    def generateCmd(self, cmd, data):
        buffer = [0] * (len(data) + 5)
        buffer[0] = data_frame.HEAD1
        buffer[1] = data_frame.HEAD2
        buffer[2] = cmd
        buffer[3] = len(data)
        for i in range(len(data)):
            buffer[4 + i] = data[i]
        # 校验位
        check = 0
        for i in range(len(buffer)):
            check += buffer[i]
        buffer[len(data) + 4] = check & 0xFF
        # for i in range(len(buffer)):
        #     print(hex(int(buffer[i])))
        return bytes(buffer)

    # 获取里程计信息
    def get_odom(self):
        data = [0] * 0
        data_pack = self.generateCmd(data_frame.ID_GET_ODOM, data)
        self.msg_list.append(data_pack)

    # 获取超声波传感器数据
    def get_ul_tof(self):
        data = [0] * 0
        data_pack = self.generateCmd(data_frame.ID_GET_UL_TOF, data)
        self.msg_list.append(data_pack)

    # 获取传感器数据
    def get_sensor(self):
        data = [0] * 0
        data_pack = self.generateCmd(data_frame.ID_GET_SENSOR, data)
        self.msg_list.append(data_pack)

    # 修改底盘形态
    def set_chassiss_type(self, type):
        data = [0] * 1
        data[0] = type
        data_pack = self.generateCmd(data_frame.ID_SET_CHASSISS_TYPE, data)
        self.msg_list.append(data_pack)

    # 清空里程计
    def clear_odom(self):
        data = [0] * 1
        data[0] = 1
        data_pack = self.generateCmd(data_frame.ID_CLEAR_ODOM, data)
        self.msg_list.append(data_pack)

    # 机器人移动封装
    def robot_move(self, speed_x, speed_y, speed_yaw):
        pos_x = speed_x.to_bytes(2, byteorder='little', signed=True)
        pos_y = speed_y.to_bytes(2, byteorder='little', signed=True)
        pos_yaw = speed_yaw.to_bytes(2, byteorder='little', signed=True)
        pos_data = pos_x + pos_y + pos_yaw
        data_pack = self.generateCmd(data_frame.ID_SET_ROBOT_SPEED, pos_data)
        self.msg_list.append(data_pack)

    # 修改舵机ID
    def update_servo_id(self, old_id, new_id):
        data = [0] * 2
        data[0] = old_id
        data[1] = new_id
        data_pack = self.generateCmd(data_frame.ID_EDIT_SERVO_ID, data)
        self.msg_list.append(data_pack)

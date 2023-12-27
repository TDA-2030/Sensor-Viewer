#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import platform
import serial.tools.list_ports
import math
from queue import Queue, Empty, Full
import threading, traceback

class WIT_IMU():
    def __init__(self) -> None:
        self.recv_queue = Queue(10)
        self.recv_queue.queue.clear()
        self.is_connected = False
        self.find_ttyUSB()

    def connect(self, port:str, baudrate:int):
        self.wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if self.wt_imu.isOpen():
            print("\033[32m串口打开...\033[0m")
        else:
            self.wt_imu.open()
            print("\033[32m打开串口成功...\033[0m")
        self.thread = threading.Thread(target=self._recv, name='imu_recv')
        self.thread.setDaemon(True)
        self.thread.start() 
        self.is_connected = True

    def disconnect(self):
        if hasattr(self, 'thread'):
            self.wt_imu.close()
            self.is_connected = False
            self.thread.join()

    # 查找 ttyUSB* 设备
    def find_ttyUSB(self):
        posts = [port.__dict__ for port in serial.tools.list_ports.comports() if "CH340" in port.description]
        print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))


    # 校验
    @staticmethod
    def checkSum(list_data, check_data):
        return sum(list_data) & 0xff == check_data


    # 16 进制转 ieee 浮点数
    @staticmethod
    def hex_to_short(raw_data):
        return list(struct.unpack("hhhh", bytearray(raw_data)))
    
    def _recv(self):
        while True:
            try:
                buff_count = self.wt_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu 失去连接，接触不良，或断线")
                break
            else:
                if buff_count > 0:
                    buff_data = self.wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        self.handleSerialData(buff_data[i])

    def get_imu_data(self):
        d = self.recv_queue.get()
        return d

    # 处理串口数据
    def handleSerialData(self, raw_data):
        global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
        angle_flag=False
        # if python_version == '2':
        #     buff[key] = ord(raw_data)
        # if python_version == '3':
        buff[key] = raw_data

        key += 1
        if buff[0] != 0x55:
            key = 0
            return
        if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
            return
        else:
            data_buff = list(buff.values())  # 获取字典所有 value
            if buff[1] == 0x51 :
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    acceleration = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                else:
                    print('0x51 校验失败')

            elif buff[1] == 0x52:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    angularVelocity = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]

                else:
                    print('0x52 校验失败')

            elif buff[1] == 0x53:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    angle_degree = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                    angle_flag = True
                else:
                    print('0x53 校验失败')
            elif buff[1] == 0x54:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    magnetometer = self.hex_to_short(data_buff[2:10])
                else:
                    print('0x54 校验失败')

            else:
                print("该数据处理类没有提供该 " + str(buff[1]) + " 的解析")
                print("或数据错误")
                buff = {}
                key = 0

            buff = {}
            key = 0
            if angle_flag:
                _data = {}
                a = {"x":angle_degree[0], "y":angle_degree[1], "z":angle_degree[2]}
                b = {"avx":angularVelocity[0], "avy":angularVelocity[1], "avz":angularVelocity[2]}
                c = {"lax":acceleration[0], "lay":acceleration[1], "laz":acceleration[2]}
                _data["imu"] = {**a, **b, **c}
                try:
                    self.recv_queue.put(_data, block=False)
                except Full:
                    pass
        #         print(
        # '''
        # 加速度(m/s²)：
        #     x轴：%.2f
        #     y轴：%.2f
        #     z轴：%.2f

        # 角速度(rad/s)：
        #     x轴：%.2f
        #     y轴：%.2f
        #     z轴：%.2f

        # 欧拉角(°)：
        #     x轴：%.2f
        #     y轴：%.2f
        #     z轴：%.2f

        # 磁场：
        #     x轴：%.2f
        #     y轴：%.2f
        #     z轴：%.2f

        # ''' % (acceleration[0], acceleration[1], acceleration[2],
        #     angularVelocity[0], angularVelocity[1], angularVelocity[2],
        #     angle_degree[0], angle_degree[1], angle_degree[2],
        #     magnetometer[0], magnetometer[1], magnetometer[2]
        #     ))



key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


if __name__ == "__main__":

    imu = WIT_IMU()
    imu.connect("/dev/ttyUSB0", 9600)
    print(f"{platform.system()}")
   
    while True:
        dd = imu.get_imu_data()
        print(dd)
        


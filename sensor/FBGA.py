from socket import socket, AF_INET, SOCK_DGRAM, timeout
import threading, traceback
from queue import Queue, Empty, Full
from typing import Tuple
import numpy as np

MIN_WAVELENGTH = 1527000
MAX_WAVELENGTH = 1568000

"""
RJ45 通讯接口初始值:
设备       解调仪(从机)       PC(主机)
IP 地址    192.168.0.10        192.168.0.X
子网掩码   255.255.255.0       255.255.255.0
默认网关   192.168.0.1         192.168.0.1
端口号     1234                4567

"""

class UDPClient():

    _lock = threading.Lock()
    def __init__(self) -> None:
        pass

    def connect(self, ip, port, server_ip=("192.168.0.10", 1234)):
        print(f"Connecting to {ip}:{port}")
        self.server_ip = server_ip
        self.udpCliSock = socket(AF_INET,SOCK_DGRAM)
        self.udpCliSock.settimeout(1)
        self.udpCliSock.bind((ip, port))

    def disconnect(self):
        self.udpCliSock.close()

    def send(self, msg: bytes):
        self.udpCliSock.sendto(msg, self.server_ip)

        # udpCliSock.close()


class FBGASensor(UDPClient):

    def __init__(self) -> None:
        super().__init__()
        self.channel_shape = (16, 32)
        self.mean = np.zeros(self.channel_shape)
        self.channel_mask = np.ones(self.channel_shape).astype(bool)

    def connect(self, ip, port, server_ip=("192.168.0.10", 1234)):
        super().connect(ip, port, server_ip)
        _d = self.FBGA_GetWaveData(False)
        self.channel_mask = _d.astype(bool)
    
    def disconnect(self):
        self.is_stream = False
        if hasattr(self, 'thread'):
            self.thread.join()
        return super().disconnect()

    def FBGA_GetDeviceInfo(self)->dict:
        self.send(b"870002")
        data = b""
        try:
            for ii in range(3):
                d, server = self.udpCliSock.recvfrom(1400)
                data += d
        except OSError:
            self._lock.release()
            return None
        if not data[0] == 135:
            raise ValueError("command error")
        deviceInfo = {}
        deviceInfo["Start_Wavelength"] = (MIN_WAVELENGTH + ( data[3]*256 + data[4] ) ) / 1000
        deviceInfo["End_Wavelength"] = (MIN_WAVELENGTH + ( data[5]*256 + data[6] ) ) / 1000
        deviceInfo["Step"] = ( data[7]*256 + data[8] ) / 1000
        deviceInfo["Channels"] = data[9]
        Peak = [0] * 16
        PV = [0] * 16
        Gain = [0] * 16
        d = data[10:]
        for i in range(16):
            Peak[i] = int.from_bytes(d[i*4:i*4+2], 'big')
            PV[i] = int.from_bytes(d[i*4+2:i*4+4], 'big')
            Gain[i] = int(d[74+i])
        deviceInfo["Peak"] = Peak
        deviceInfo["PV"] = PV
        deviceInfo["Gain"] = Gain
        return deviceInfo

    def FBGA_GetSpectrumData(self, channel:int)->Tuple[int, np.ndarray]:
        if channel > 8:
            print("only support 8 channel")
            return
        self.send(b"580003"+bytes(f"{channel:02d}", encoding='utf-8'))
        data = b""
        try:
            for ii in range(3):
                d, server = self.udpCliSock.recvfrom(1400)
                data += d
        except OSError:
            self._lock.release()
            return None
        if not data[0] == 88:
            raise ValueError("command error")
        # d = np.zeros((1024,))
        d = np.frombuffer(data[4:4106], dtype='>u2')
        ch = int(data[3])
        # data = data[4:]
        # for i in range(d.size):
        #     d[i] = int.from_bytes(data[i*2:(i*2)+2], 'big')
        return ch, d

    def FBGA_GetWaveData(self, diff:bool=True)->np.ndarray:
        data = b""
        self._lock.acquire()
        self.send(b"570002")
        try:
            for ii in range(3):
                d, server = self.udpCliSock.recvfrom(1400)
                data += d
        except OSError:
            self._lock.release()
            return self.mean
        if not data[0] == 87:
            raise ValueError("command error")
        data = data[3:]
        d = self.to_wavedata(data)
        if diff:
            d -= self.mean
        self._lock.release()
        return d

    def FBGA_GetStreamWaveData(self)->np.ndarray:
        data = self.recv_queue.get()
        return self.to_wavedata(data)

    def to_wavedata(self, data, diff:bool=True):
        d = np.zeros(self.channel_shape)
        for i in range(d.shape[0]):
            for j in range(d.shape[1]):
                c = i*32+j
                d[i][j] = int.from_bytes(data[c*3:c*3+3], 'big') / 10 # 装换为pm单位
        return d

    def FBGA_tare(self):
        self.mean = self.FBGA_GetWaveData(False).copy()

    def FBGA_WaveMode(self, is_contiune:bool):
        if is_contiune:
            self.recv_queue = Queue(10)
            self.recv_queue.queue.clear()
            self.thread = threading.Thread(target=self._recv, name='udprecv', daemon=True)
            self.is_stream = True
            self.thread.start() 
        else:
            self.is_stream = False
            if hasattr(self, 'thread'):
                self.thread.join()
        self.send(b"560003FF" if is_contiune else b"56000300")

    def _recv(self):
        while self.is_stream:
            try:
                data = b""
                for ii in range(3):
                    d, server = self.udpCliSock.recvfrom(1400)
                    data += d
                self.recv_queue.put(data, block=False)
            except Full:
                time.sleep(0.1)
                # print("queue full")
            except OSError:
                break
            except Exception as e:
                traceback.print_exc()
                break

    def FBGA_ConvertForce(self, wavedata:np.ndarray)->dict:
        out = {}
        # 波长转换为力值
        return out


def show_SpectrumData(Start_Wavelength:float, End_Wavelength:float):
    import matplotlib.pyplot as plt

    info = s.FBGA_GetSpectrumData(1)

    chunk_size = info[1].size
    # 初始化时间轴和数据缓冲区
    t = np.linspace(Start_Wavelength, End_Wavelength, chunk_size)
    data = np.zeros(chunk_size)

    # 创建频谱图窗口和曲线对象
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot(t, data)
    # ax.set_xlabel('Time (s)')
    ax.set_ylabel('Amplitude')
    ax.set_ylim(0, 20000)


    # 实时更新波形图
    while True:
        info = s.FBGA_GetSpectrumData(1)
        data = info[1]
        wl = s.FBGA_GetWaveData()
        print(f"ch:{info[0]}, max:{data.max()}, Wave data:{wl[np.where(s.channel_mask==True)]}")


        # 更新波形图
        line.set_ydata(data)
        fig.canvas.draw()
        fig.canvas.flush_events()

if __name__ == '__main__':
    s = FBGASensor()
    s.connect('192.168.0.111', 4567, server_ip=("192.168.0.10", 1234))
    dev_info = s.FBGA_GetDeviceInfo()
    print(f"info:{dev_info}")
    import time 

    try:
        show_SpectrumData(dev_info['Start_Wavelength'], dev_info['End_Wavelength'])
        exit()
        cnt = 0
        last_t = time.perf_counter()
        s.FBGA_WaveMode(False)
        # s.FBGA_tare()
        print(s.channel_mask)
        while True:
            info = s.FBGA_GetWaveData()
            print(f"Wave data:{info[np.where(s.channel_mask==True)]}")
            # time.sleep(0.001)
            cnt += 1
            if cnt > 50:
                _t = time.perf_counter()
                print(f"Rate: {cnt/(_t - last_t):.2f} samples/sec")
                last_t = _t
                cnt = 0
    except KeyboardInterrupt as e:
        s.disconnect()


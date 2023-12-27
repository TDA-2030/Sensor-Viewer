from socket import socket, AF_INET, SOCK_DGRAM, timeout
import threading, traceback
from queue import Queue, Empty, Full
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

    def __init__(self) -> None:
        pass

    def connect(self, ip, port, server_ip=("192.168.0.10", 1234)):
        print(f"Connecting to {ip}:{port}")
        self.server_ip = server_ip
        self.udpCliSock = socket(AF_INET,SOCK_DGRAM)
        self.udpCliSock.settimeout(1)
        self.udpCliSock.bind((ip, port))

        self.recv_queue = Queue(10)
        self.recv_queue.queue.clear()
        self.thread = threading.Thread(target=self._recv, name='udprecv')
        self.thread.setDaemon(True)
        self.thread.start() 

    def disconnect(self):
        self.udpCliSock.close()
        if hasattr(self, 'thread'):
            self.thread.join()

    def _recv(self):
        data = b""
        packages = 0
        while True:
            try:
                d, server = self.udpCliSock.recvfrom(4200)
                data += d
                packages += 1
                if packages == 3:
                    self.recv_queue.put(data, block=False)
                    data = b""
                    packages = 0
            except Full:
                pass
                # print("queue full")
            except OSError:
                break
            except Exception as e:
                traceback.print_exc()
                break


    def send(self, msg: bytes, wait_reply:bool=True):
        self.udpCliSock.sendto(msg, self.server_ip)
        if not wait_reply:
            return
        try:
            data: str = self.recv_queue.get(block=True, timeout=2)
            return data
        except Empty:
            print("command no ack")
        return 

        # udpCliSock.close()


class FBGASensor(UDPClient):

    def __init__(self) -> None:
        super().__init__()
        self.mean = np.zeros((16, 32))

    def FBGA_GetDeviceInfo(self)->dict:
        data = self.send(b"870002")
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
            Gain[i] = int.from_bytes(d[74+i], 'big')
        deviceInfo["Peak"] = Peak
        deviceInfo["PV"] = PV
        deviceInfo["Gain"] = Gain
        return deviceInfo

    def FBGA_GetSpectrumData(self, channel:int):
        data = self.send(b"580003"+channel.to_bytes(1, 'big', signed=False))
        if not data[0] == 88:
            raise ValueError("command error")
        d = np.zeros(2051)
        ch = int.from_bytes(data[3], 'big')
        for i in range(d.size):
            d[i] = int.from_bytes(data[4+i:6+i], 'big')
        return ch, d
    
    def FBGA_GetWaveData(self, diff:bool=True)->np.ndarray:
        data = self.send(b"570002")
        if not data[0] == 87:
            raise ValueError("command error")
        data = data[3:]
        d = np.zeros((16, 32))
        for i in range(d.shape[0]):
            for j in range(d.shape[1]):
                c = i*32+j
                d[i][j] = int.from_bytes(data[c*3:c*3+3], 'big')
        return d - self.mean if diff else d
    
    def FBGA_ConvertForce(self, wavedata:np.ndarray)->dict:
        out = {}
        out["fx"] = wavedata[0][0]/1000
        out["fy"] = wavedata[1][0]/1000
        return out
    
    def FBGA_WaveMode(self, is_contiune:bool):
        self.send(b"560003FF" if is_contiune else b"56000300", wait_reply=False)

    def FBGA_tare(self):
        self.mean = self.FBGA_GetWaveData(False).copy()


if __name__ == '__main__':
    s = FBGASensor()
    s.connect('192.168.0.111', 4567, server_ip=("192.168.0.10", 1234))
    import time 

    while True:

        info = s.FBGA_GetWaveData()
        print("wave data: ", info[1])
        time.sleep(0.1)
        # break

    

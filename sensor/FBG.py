from socket import socket, AF_INET, SOCK_DGRAM, timeout
import threading, traceback
from queue import Queue, Empty, Full
import json

"""
解调器服务模式程序端口

TCP: 7090
UDP: 7082
Websocket: 7091

TCP为短连接，服务端接受指令并返回数据后，将主动断开网络连接。
Websocket为长连接，服务端不主动断开客户端网络连接。

"""

class UDPServer():

    def __init__(self) -> None:
        pass

    def connect(self, ip, port):
        print(f"Connecting to {ip}:{port}")
        self.udpCliSock = socket(AF_INET,SOCK_DGRAM)
        self.udpCliSock.settimeout(1)
        self.udpCliSock.bind((ip, port))
        # self.udpCliSock.sendto(b"sdfdfg", (self.ip,self.port))

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
        while True:
            try:
                data, server = self.udpCliSock.recvfrom(1024)
                data:bytes
                if data:
                    data = data.decode("utf-8")
                    # print(data)  # id, ch, wavelength, PhysicValue 
                    # b'{"Time":13432,"SN":"6910002000","Counter":925053,"Data":[[1,1,1550393,-22.000000,6820020,0,0]]}'
                    data = json.loads(data)
                    self.recv_queue.put(data, block=False)

            except Full:
                pass
                # print("queue full")
            except OSError:
                break
            except Exception as e:
                traceback.print_exc()
                break


    def send(self, msg: bytes, wait_reply:bool=True):
       
        self.udpCliSock.sendto(msg, (self.ip,self.port))
        if not wait_reply:
            return
        try:
            data: str = self.recv_queue.get(block=True, timeout=10)
            data = data.decode().strip()
            return data
        except Empty:
            print("command no ack")
        return 

        # udpCliSock.close()


class FBGSensor(UDPServer):

    CH_0 = 0x01
    CH_1 = 0x02
    CH_2 = 0x04
    CH_3 = 0x08
    CH_4 = 0x10
    CH_5 = 0x20
    CH_6 = 0x40
    CH_7 = 0x80

    def __init__(self) -> None:
        super().__init__()
        self.mean = [0] * 128

    @property
    def version(self):
        data = self.udpserver.send(b":SYS:VER?")
        data = json.load(data)
        return data
    
    def get_sensor_info(self):
        data = self.udpserver.send(b":FBG:CFG?")
        print(data)
        data = json.load(data)

        return data
    
    @staticmethod
    def _icToIndex(ch, id)->int:
        return 16*ch+id
    
    def get_sensor_data(self)->dict:
        data = self.recv_queue.get(timeout=1)['Data']
        for i, d in enumerate(data):
            ch = d[1]
            id = d[0]
            _d = d[3]
            data[i][3] = _d - self.mean[self._icToIndex(ch, id)]
        return data
    
    def tare(self):
        data = self.recv_queue.get()['Data']
        for i, d in enumerate(data):
            ch = d[1]
            id = d[0]
            _d = d[3]
            self.mean[self._icToIndex(ch, id)] = _d


if __name__ == '__main__':
    
    s = FBGSensor()
    s.connect('192.168.1.101', 7102)
    # print(s.version)
    import time 

    while True:
        time.sleep(1)
        # s.get_sensor_data(FBGSensor.CH_0|FBGSensor.CH1)
        # break

    

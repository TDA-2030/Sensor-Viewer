import sys
from pathlib import Path
import time
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import FBG
import NetFT
import ZNBSQ
import numpy as np
from collections import OrderedDict
import FBGA
from functools import wraps
import types

class CallTimer:
    def __init__(self, func):
        wraps(func)(self)
        self.call_count = 0
        self.last_t = time.perf_counter()

    def __call__(self, *args, **kwargs):
        result = self.__wrapped__(*args, **kwargs)
        self.call_count += 1
        if self.call_count > 100:
            t = time.perf_counter()
            print(f"Sample rate: {100/(t-self.last_t):.2f}")
            self.last_t = t
            self.call_count = 0

        return result
    
    def __get__(self, instance, cls):
        if instance is None:
            return self
        else:
            return types.MethodType(self, instance)



class FSensors():

    def __init__(self) -> None:
        self.call_count = 0
        self.last_t = time.perf_counter()

    def fs_connect_fbg(self, ip:str, port:int):
        self.fbg = FBG.FBGSensor()
        self.fbg.connect(ip, int(port))

    def fs_connect_ati(self, ip:str):
        self.ati = NetFT.Sensor(ip)
        self.ati.tare()
        self.ati_CPF = np.array([1000000,1000000,1000000,1000000,1000000,1000000])

    def fs_connect_zn(self, port:str, buad:int, addr:int=1):
        com = ZNBSQ.COM(port, int(buad))
        self.znbsq = ZNBSQ.ForceSensor(com, addr)

    def fs_connect_fbga(self, ip:str, port:int):
        self.fbga = FBGA.FBGASensor()
        self.fbga.connect(ip, port, server_ip=("192.168.0.10", 1234))

    def fs_disconnect(self):
        if hasattr(self, "fbg"):
            self.fbg.disconnect()
            del self.fbg
        if hasattr(self, "ati"):
            if self.ati.stream:
                self.ati.stopStreaming()
                self.ati.thread.join()
            self.ati.sock.close()
            del self.ati
        if hasattr(self, "znbsq"):
            self.znbsq.com.close()
            del self.znbsq

        if hasattr(self, "fbga"):
            self.fbga.disconnect()
            del self.fbga

    def fs_get_data(self)->dict: 

        data = OrderedDict()
        if hasattr(self, "fbg"):
            temp=OrderedDict()
            _data = self.fbg.get_sensor_data()
            for ch, d in enumerate(_data):
                name = f'ID{d[0]}-CH{d[1]}'
                temp[name] = float(d[3])
            for (key,value) in self.fbg.FBG_ConvertForce(_data).items():
                temp[f"Force_{key}"] = value
            data["fbg"] = temp
        if hasattr(self, "ati"):
            d = self.ati.getMeasurement()
            d = np.array(d) / self.ati_CPF
            print(d)
            data["ati"] = {"Fx":d[0], "Fy":d[1], "Fz":d[2], "Tx":d[3], "Ty":d[4],"Tz":d[5]}

        if hasattr(self, "znbsq"):
            temp = self.znbsq.get_force()
            data["znbsq"] = {"Fz":float(temp)}

        if hasattr(self, "fbga"):
            temp=OrderedDict()
            _data = self.fbga.FBGA_GetWaveData()
            idx = np.where(self.fbga.channel_mask==True)
            for x, y in zip(*idx):
                name = f'CH{x}-{y}'
                temp[name] = _data[x, y]
            for (key,value) in self.fbga.FBGA_ConvertForce(_data).items():
                temp[f"Force_{key}"] = value
            data["fbga"] = temp

        self.call_count += 1
        if self.call_count > 200:
            t = time.perf_counter()
            print(f"Sample rate: {200/(t-self.last_t):.2f}")
            self.last_t = t
            self.call_count = 0

        return data

if __name__ == "__main__":
    fbga = FSensors()

    for i in range(200):
        time.sleep(0.001)
        print(fbga.fs_get_data())
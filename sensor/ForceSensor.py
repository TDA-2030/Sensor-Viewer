import sys
from pathlib import Path

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

class FSensors():

    def __init__(self) -> None:
        pass

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

    def fs_get_data(self, timeout=None)->dict: 

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
            for i, d in enumerate(_data):
                if d[0] !=0:
                    name = f'CH{i}'
                    temp[name] = d[0]
            for (key,value) in self.fbga.FBGA_ConvertForce(_data).items():
                temp[f"Force_{key}"] = value
            data["fbga"] = temp
        return data


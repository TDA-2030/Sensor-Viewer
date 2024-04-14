
from typing import List
import serial
import time
import numpy as np
from sensor import simbatouch
from motor.controller import MotionController
from motor.demo_driver import get_serial_port
from motor import demo_driver

if __name__ == "__main__":

    for c in get_serial_port():
        print(c)

    ser = serial.Serial("COM11", 115200, timeout=0.5)
    com = simbatouch.RS485(ser, 1)
    com.connect()
    true_sensor = simbatouch.SBT906D(com)
    motion:List[MotionController] = []
    ser = serial.Serial("COM9", 115200, timeout=0.5)
    for addr, screw_d, ch in zip((2, 3), (1, 6), (5, 4)):
        try:
            print("--------------", addr, screw_d, ch)
            com = demo_driver.RS485(ser, addr)
            com.connect()
            _mot = MotionController(com, screw_d)
            _mot.attach_sensor_get_cb(true_sensor.get_force, ch)
            motion.append(_mot)
        except:
            print(f"电机id{addr} 连接失败")

    try:
        _id = 1
        s = 100
        e = 600
        n = 5
        interval = 2.
        steps = np.linspace(s, e, n, endpoint=True)
        motion[_id].start()
        for f in steps:
            print(f"Current set force to {f:.1f}g")
            motion[_id].set_target_force(f)
            time.sleep(interval)
            print(f"{motion[_id].v:.1f} err={motion[_id].v-f:.1f}")
    except KeyboardInterrupt:
        print("exit")
    motion[_id].stop()

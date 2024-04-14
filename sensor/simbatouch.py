import time
from queue import Queue, Empty
from threading import Lock
import minimalmodbus
import serial
import serial.tools.list_ports
import traceback
from math import pow


class RS485:
    _lock = Lock()

    def __init__(self, serial:serial.Serial, slave_addr=6) -> None:
        self.ser = serial
        self.slave_addr = slave_addr
        self.is_connected: bool = False

    def connect(self):
        try:
            print(f"Connecting RS485 addr:{self.slave_addr}")
            # self.master = modbus_tcp.TcpMaster(ip, port)
            # self.master.set_timeout(5.0)
            self.master = minimalmodbus.Instrument(self.ser, self.slave_addr)

            self.is_connected = True
            return True
        except:
            traceback.print_exc()
            return False

    def disconnect(self):
        self.ser.close()
        del self.master
        del self.ser
        self.is_connected = False

    def read_register(self, addr: int):
        self._lock.acquire()
        ret = self.master.read_register(addr)
        self._lock.release()
        return ret

    def write_register(self, addr: int, value: int):
        self._lock.acquire()
        ret = self.master.write_register(addr, value, functioncode=16)
        self._lock.release()
        return ret

    def write_long(self, addr: int, value: int, number_of_bytes: int = 2):
        self._lock.acquire()
        ret = self.master.write_long(addr, value, signed=True, byteorder=minimalmodbus.BYTEORDER_BIG, number_of_registers=number_of_bytes)
        self._lock.release()
        return ret

    def read_long(self, addr: int, number_of_bytes: int = 2):
        self._lock.acquire()
        ret = self.master.read_long(addr, signed=True, byteorder=minimalmodbus.BYTEORDER_BIG, number_of_registers=number_of_bytes)
        self._lock.release()
        return ret


class SBT906D:

    def __init__(self, com: RS485):
        self.com = com
        self.mean = [0] * 6
        # calculate factor by point position
        self.factor = 10#pow(10, self.get_status() & 0x7)
        print(f"factor:{self.factor}")
        self.set_manual_zero_range(20)

    def get_force(self, channel: int) -> float:
        addr = 500 * channel + 80
        res = self.com.read_long(addr)
        return float(res / self.factor) - self.mean[channel]

    def tare(self, channel: int) -> None:
        addr = 500 * channel + 80
        res = self.com.read_long(addr)
        res = float(res / self.factor)
        self.mean[channel] = res

    def set_buad(self, buad: int):
        self.com.write_register(5, 0x5AA5)  # unlock
        d = {9600: 3, 19200: 4, 38400: 5, 57600: 6, 115200: 7, 230400: 8, 460800: 9, 921600: 10}
        self.com.write_register(1, d[buad])
        self.com.write_register(5, 1)  # lock

    def set_manual_zero_range(self, range):
        self.com.write_register(93, range)

    def set_manual_zero_range(self, range):
        self.com.write_register(95, range)

    def get_manual_zero_range(self):
        return self.com.read_register(93)

    def get_auto_zero_range(self):
        return self.com.read_register(95)

    def get_status(self):
        return self.com.read_register(8)

    def get_measure_value(self):
        return self.com.read_long(30)

    def get_AD_speed(self):
        return self.com.read_register(32)

    def get_polarity(self):
        return self.com.read_register(33)

    def get_filter(self):
        return self.com.read_register(34)

    def get_zero_measure_value(self):
        return self.com.read_long(38)

    def get_gain_measure_value(self):
        return self.com.read_long(42)

    def get_sensitivity(self):
        return self.com.read_long(46)

    def get_range(self):
        return self.com.read_long(48)

    def show_info(self):
        print(f"status:{self.get_status()}")
        print(f"measure_value:{self.get_measure_value()}")
        print(f"polarity:{self.get_polarity()}")
        print(f"AD_speed:{self.get_AD_speed()}")
        print(f"range:{self.get_range()}")
        print(f"zero_measure_value:{self.get_zero_measure_value()}")
        print(f"manual_zero_range:{self.get_manual_zero_range()}")
        print(f"auto_zero_range:{self.get_auto_zero_range()}")
        print(f"sensitivity:{self.get_sensitivity()}")


if __name__ == "__main__":

    ser = serial.Serial("COM4", 115200, timeout=0.5)
    com = RS485(ser, 1)
    com.connect()
    s = SBT906D(com)
    # s.set_buad(115200)
    # s.tare(4)
    # s.tare(5)
    time.sleep(0.1)
    s.show_info()
    # exit()
    try:
        cnt = 0
        last_t = time.perf_counter()
        while True:
            f1 = s.get_force(4)
            f2 = s.get_force(5)
            print(f"{f1:.3f}, {f2:.3f}")
            time.sleep(0.001)
            cnt += 1
            if cnt > 50:
                _t = time.perf_counter()
                print(f"Rate: {cnt/(_t - last_t):.2f} samples/sec")
                last_t = _t
                cnt = 0
    except KeyboardInterrupt as e:
        com.disconnect()

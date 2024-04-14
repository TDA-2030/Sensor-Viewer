from typing import Callable
import serial
import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
PRINT = print
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

from demo_driver import LeadScrew, RS485
import time
from simple_pid import PID
from threading import Thread

class MotionController:

    def __init__(self, com: RS485, screw_d: float, name: str = "") -> None:
        self.motor = LeadScrew(com, screw_d, name)
        self.motor.reverse_direction(True)
        self.motor.set_rated_current(0.5)
        self.motor.set_voltage_threshold_max(25.5)
        self.motor.show_info()
        self.pid = PID(0.15, 0.0007, 0.010, setpoint=0)
        self.pid.output_limits = (-20, 20)
        self.v = 0.0
        self.interval = 1/40
        self.is_run = False
        self.sensor_get_cb = None

    def attach_sensor_get_cb(self, get_force: Callable, *args):
        self.sensor_get_cb = get_force
        self.sensor_get_cb_args = args

    def _run_pid(self):
        while self.is_run:
            starttime = time.perf_counter()
            self.v = self.sensor_get_cb(*self.sensor_get_cb_args)
            # Compute new output from the PID according to the systems current value
            output = self.pid(self.v)
            # print(f"\rin={self.v}, pid out={output:.3f}", end="", flush=True)
            try:
                self.motor.set_velocity(output)
            except:
                print("motor error")
            time.sleep(max(0, self.interval - (time.perf_counter() - starttime)))

    def start(self):
        if self.is_run: return
        if self.sensor_get_cb is None: 
            print("No sensor get callback")
            return
        self.thread_pid = Thread(target=self._run_pid, daemon=True)
        self.is_run = True
        self.thread_pid.start()

    def stop(self):
        if not self.is_run: return
        self.is_run = False
        self.thread_pid.join()
        self.motor.enable_motor(False)

    def set_target_force(self, force: float):
        self.pid.setpoint = force
        self.pid._integral = 0

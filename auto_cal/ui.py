import sys
from typing import Callable, Dict, List
from PyQt5.QtWidgets import QDialog, QApplication, QVBoxLayout, QMainWindow, QMessageBox, QWidget
from PyQt5.QtGui import QIcon, QPixmap, QFont, QCloseEvent
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer

import pyqtgraph as pg
import pyqtgraph.exporters
import seaborn as sns
import numpy as np
import time, traceback
from pathlib import Path
from queue import Queue, Empty
from threading import Thread, Event, current_thread

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
PRINT = print
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

if str(ROOT/"..") not in sys.path:
    sys.path.append(str(ROOT/".."))

import serial
import mainwin_ac
from sensor import ForceSensor
from sensor import simbatouch
from motor import demo_driver
from motor.controller import MotionController
from motor.demo_driver import get_serial_port


class MainWin(QMainWindow):

    def __init__(self, _sensor: ForceSensor.FSensors, parent=None):
        super(QMainWindow, self).__init__(parent)
        self.ui = mainwin_ac.Ui_MainWindow()
        self.ui.setupUi(self)

        self.fsensors = _sensor

        self.ui.comboBox_mot_port.clear()
        self.ui.comboBox_sen_port.clear()

        _port = get_serial_port()
        _correct_port = scan_correct_port(_port)
        for _p in _port:
            self.ui.comboBox_mot_port.addItem(_p["device"])
            self.ui.comboBox_sen_port.addItem(_p["device"])
        if "motor" in _correct_port:
            self.ui.comboBox_mot_port.setCurrentText(_correct_port["motor"])
        if "sensor" in _correct_port:
            self.ui.comboBox_sen_port.setCurrentText(_correct_port["sensor"])
        self.dual_mode = False

        self.ui.scrollArea.setEnabled(False)
        self.ui.pushButton_cal_lm.pressed.connect(lambda: self.btn_move_right(True, 0))
        self.ui.pushButton_cal_lm.released.connect(lambda: self.btn_move_right(False, 0))
        self.ui.pushButton_cal_lp.pressed.connect(lambda: self.btn_move_left(True, 0))
        self.ui.pushButton_cal_lp.released.connect(lambda: self.btn_move_left(False, 0))
        self.ui.pushButton_cal_lh.clicked.connect(lambda: self.btn_home_callback(0))
        self.ui.pushButton_cal_rm.pressed.connect(lambda: self.btn_move_right(True, 1))
        self.ui.pushButton_cal_rm.released.connect(lambda: self.btn_move_right(False, 1))
        self.ui.pushButton_cal_rp.pressed.connect(lambda: self.btn_move_left(True, 1))
        self.ui.pushButton_cal_rp.released.connect(lambda: self.btn_move_left(False, 1))
        self.ui.pushButton_cal_rh.clicked.connect(lambda: self.btn_home_callback(1))

        self.ui.pushButton_cal_start.clicked.connect(self.btn_cal_start)
        self.ui.horizontalSlider_speed.valueChanged.connect(self._solt_vchanged)
        self._solt_vchanged()

        self.ui.tabWidget.setCurrentIndex(1)
        self.ui.pushButton_connect.clicked.connect(self._solt_connect)
        self.ui.pushButton_fbgconfig.clicked.connect(self._solt_fbgconfig)
        self.ui.pushButton_fbga_config.clicked.connect(self._solt_fbgaconfig)

        pg.setConfigOptions(leftButtonPan=True, antialias=True)
        pg.setConfigOption("background", (30, 30, 33))
        pg.setConfigOption("foreground", "w")

        self.ui.horizontalLayout = QVBoxLayout(self.ui.groupBox_chart)
        self.ui.horizontalLayout.setObjectName("verticalLayout")
        self.ui.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.pg_win = pg.GraphicsLayoutWidget(self.ui.groupBox_chart, show=True)
        self.ui.horizontalLayout.addWidget(self.pg_win)
        self.pw = []
        pw: pg.PlotItem = self.pg_win.addPlot(title="Raw data")
        labelStyle = {"color": "#ffffff", "font-size": "16pt"}
        pw.getAxis("left").setLabel("PhysicValue", units="pm", **labelStyle)
        self.pw.append(pw)
        self.pg_win.nextRow()
        pw = self.pg_win.addPlot(title="Force Value")
        pw.getAxis("left").setLabel("Force", units="g", **labelStyle)
        self.pw.append(pw)
        self.pw: List[pg.PlotItem]

        for pw in self.pw:
            pw.showGrid(x=True, y=True, alpha=0.4)
            pw.addLegend()

            left_axis = pw.getAxis("left")
            bottom_axis = pw.getAxis("bottom")
            # left_axis.enableAutoSIPrefix(False)
            font = QFont()
            font.setPixelSize(16)
            left_axis.tickFont = font
            bottom_axis.tickFont = font
            bottom_axis.setLabel("Time", units="sec", **labelStyle)

    def _solt_connect(self):
        if self.ui.pushButton_connect.text() == "连接":
            print("start")
            self.ui.pushButton_connect.setText("断开")
            self.ui.pushButton_connect.setEnabled(False)
            self.ui.pushButton_connect.setIcon(QIcon(QPixmap(":/main/disconnect.png")))

            if self.ui.checkBox_fbg.isChecked():
                text = self.ui.ipaddress_edit.text().strip()
                text = text.split(":")
                try:
                    self.fsensors.fs_connect_fbg(text[0], int(text[1]))
                except:
                    traceback.print_exc()
                    QMessageBox.warning(self, "Warning", "无法连接解调仪，请检查连接是否正确", buttons=QMessageBox.Ok)
                    return
            if self.ui.checkBox_fbga.isChecked():
                text = self.ui.lineEdit_fbga.text().strip()
                text = text.split(":")
                try:
                    self.fsensors.fs_connect_fbga(text[0], int(text[1]))
                except:
                    self.fsensors.fs_disconnect()
                    traceback.print_exc()
                    QMessageBox.warning(self, "Warning", "无法连接FBGA解调仪，请检查连接是否正确", buttons=QMessageBox.Ok)
                    return
            text = self.ui.comboBox_sen_port.currentText().strip()
            try:
                com = simbatouch.RS485(serial.Serial(text, 115200, timeout=0.5), 1)
                com.connect()
                self.true_sensor = simbatouch.SBT906D(com)
            except:
                self.fsensors.fs_disconnect()
                traceback.print_exc()
                QMessageBox.warning(self, "Warning", "无法通过 RS485 连接力传感器，请检查连接是否正确", buttons=QMessageBox.Ok)
                return

            text = self.ui.comboBox_mot_port.currentText().strip()
            
            self.motion:List[MotionController] = []
            ser = serial.Serial(text, 115200, timeout=0.5)
            for addr, screw_d, ch in zip((2, 3), (1, 6), (5, 4)):
                try:
                    com = demo_driver.RS485(ser, addr)
                    com.connect()
                    _mot = MotionController(com, screw_d)
                    _mot.attach_sensor_get_cb(self.true_sensor.get_force, ch)
                    self.motion.append(_mot)
                except:
                    print(f"电机id{addr} 连接失败")

            if len(self.motion) == 0:
                self.fsensors.fs_disconnect()
                self.true_sensor.com.disconnect()
                traceback.print_exc()
                QMessageBox.warning(self, "Warning", "无法通过 RS485 连接电机，请检查连接是否正确", buttons=QMessageBox.Ok)
                return

            self.ui.scrollArea.setEnabled(True)
            self.ui.comboBox_mot_port.setEnabled(False)
            self.ui.comboBox_sen_port.setEnabled(False)
            self.ui.comboBox_cal_mode.clear()
            self.ui.comboBox_cal_mode.addItem("单边")
            if len(self.motion) > 1:
                self.ui.comboBox_cal_mode.addItem("双边")

            if self.ui.checkBox_fbg.isChecked():
                self.ui.ipaddress_edit.setEnabled(False)
            if self.ui.checkBox_fbga.isChecked():
                self.ui.lineEdit_fbga.setEnabled(False)

            self.ui.checkBox_fbg.setEnabled(False)
            self.ui.checkBox_fbga.setEnabled(False)
        else:
            print("stop")
            self.ui.pushButton_connect.setText("连接")
            self.ui.pushButton_connect.setIcon(QIcon(QPixmap(":/main/connect.png")))

            if self.ui.checkBox_fbg.isChecked():
                self.ui.ipaddress_edit.setEnabled(True)
            if self.ui.checkBox_fbga.isChecked():
                self.ui.lineEdit_fbga.setEnabled(True)

            self.ui.comboBox_mot_port.setEnabled(True)
            self.ui.comboBox_sen_port.setEnabled(True)
            self.ui.checkBox_fbg.setEnabled(True)
            self.ui.checkBox_fbga.setEnabled(True)
            self.fsensors.fs_disconnect()
            self.ui.scrollArea.setEnabled(False)

    def _solt_fbgconfig(self):
        self.fsensors.fbg.tare()

    def _solt_fbgaconfig(self):
        self.fsensors.fbga.FBGA_tare()

    def _solt_vchanged(self):
        self.speed = self.ui.horizontalSlider_speed.value() / 10
        self.ui.label_cal_set_speed.setText(f"{self.speed}mm/s")

    def btn_move_right(self, act: bool, mot_id: int):
        if not act:
            self.motion[mot_id].motor.set_velocity(0)
        else:
            self.motion[mot_id].motor.set_velocity(self.speed)

    def btn_move_left(self, act: bool, mot_id: int):
        if not act:
            self.motion[mot_id].motor.set_velocity(0)
        else:
            self.motion[mot_id].motor.set_velocity(-self.speed)

    def btn_home_callback(self, mot_id: int):
        pass
        # self.motion[mot_id].motor.check_home(False)

    def btn_cal_start(self):

        if self.ui.pushButton_cal_start.text() == "开始":
            self.ui.pushButton_cal_start.setText("停止")
            self.dual_mode = True if self.ui.comboBox_cal_mode.currentIndex() == 1 else False
            self.data_queue = Queue(1)
            self.thread_cal = Thread(target=self._run_cal, daemon=True)
            self.exit_event = Event()
            self.thread_cal.start()

            self.prl = PlotRollLine(self.pw)
            self.chartthread = ChartThread(self.prl._init, self._get_data, self)
            self.chartthread.sinOut.connect(self.prl.drawLine)
            self.chartthread.start()
            for pw in self.pw:
                pw.enableAutoRange(axis="y", enable=True)
        else:
            self.motion[0].stop()
            if self.dual_mode:
                self.motion[1].stop()
            self.ui.pushButton_cal_start.setText("开始")
            self.exit_event.set()

            # 放置一个空数据到最后
            self.put_empty()

            self.chartthread.stopRun()
            del self.chartthread
            if self.ui.checkBox_autosave.isChecked():
                for i, pw in enumerate(self.pw):
                    ex = pyqtgraph.exporters.CSVExporter(pw)
                    fname = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime()) + f"-plot{i}.csv"
                    ex.export(fname)

    def _run_cal(self):

        s = self.ui.doubleSpinBox_start_force.value()
        e = self.ui.doubleSpinBox_end_force.value()
        n = self.ui.spinBox_point_num.value()
        repeat = self.ui.spinBox_repeat.value()
        interval = self.ui.doubleSpinBox_interval.value()
        steps = list(np.linspace(s, e, n, endpoint=True))
        steps += list(np.linspace(e, s, n, endpoint=True))

        # 提前放置一个空数据，给绘图提供数据格式
        self.put_empty()

        self.motion[0].start()
        if self.dual_mode:
            self.motion[1].start()

        for ii in range(repeat):
            for f in steps:
                print(f"Current set force to {f:.1f} g")
                self.motion[0].set_target_force(f)
                if self.dual_mode:
                    self.motion[1].set_target_force(f)
                for _ in range(10):
                    self.exit_event.wait(interval/10)
                    sensor_d = self.fsensors.fs_get_data()
                    if self.dual_mode:
                        sensor_d.update({"SG": {"true": self.motion[0].v, "true2": self.motion[1].v}})
                        print(f"force error:{self.motion[0].v-f:.1f} {self.motion[1].v-f:.1f}")
                    else:
                        sensor_d.update({"SG": {"true": self.motion[0].v}})
                        print(f"force error:{self.motion[0].v-f:.1f}")
                    self.data_queue.put(sensor_d, block=False)
                    if self.exit_event.is_set():
                        return
    
        self.btn_cal_start()  # 停止校准

    def put_empty(self):
        sensor_d = self.fsensors.fs_get_data()
        if self.dual_mode:
            sensor_d.update({"SG": {"true": 0, "true2": 0}})
        else:
            sensor_d.update({"SG": {"true": 0}})
        self.data_queue.put(sensor_d, block=False)

    def _get_data(self) -> Dict[str, Dict[str, float]]:
        _d = self.data_queue.get()
        return _d


class PlotRollLine:
    def __init__(self, pw: List[pg.PlotWidget]) -> None:
        self.pw = pw
        for pw in self.pw:
            pw.setXRange(-5, 0)

    def _init(self, _get_data: Callable):
        print("------------")
        # 读一次数据，判断有多少个通道
        _d = _get_data()
        id = 0
        self.plots = {}

        palette = sns.color_palette(None, 32)
        palette = (np.array(palette) * 255).tolist()

        lines = {
            "imu": Qt.DashLine,
            "fbg": Qt.DotLine,
            # "fbg": Qt.SolidLine,
        }

        for sensor, svalue in _d.items():
            for key, value in svalue.items():
                key: str
                penstate_r = pg.mkPen(width=2, color=palette[id], style=lines.get(sensor, Qt.SolidLine))  # 画笔
                if key.startswith("true"):
                    self.plots[sensor + key] = self.pw[1].plot(pen=penstate_r, name=sensor + "-" + key)
                else:
                    self.plots[sensor + key] = self.pw[0].plot(pen=penstate_r, name=sensor + "-" + key)
                id += 1
        if len(self.plots) == 0:  # 没有可用通道，直接退出
            print("no data to plot")
            return
        self.plots: Dict[str, pg.PlotDataItem]

        self.startTime = time.perf_counter()
        self.ptr5 = 0
        self.data5 = np.empty((len(self.plots), 256, 2), dtype=np.float32)

    def drawLine(self, data):
        id = 0
        for sensor, svalue in data.items():
            for key, value in svalue.items():
                curve: pg.PlotDataItem = self.plots[sensor + key]
                now = time.perf_counter()
                self.data5[id, self.ptr5, 0] = now - self.startTime
                self.data5[id, self.ptr5, 1] = value  # random.random()*100
                curve.setData(x=self.data5[id, : self.ptr5, 0], y=self.data5[id, : self.ptr5, 1])
                curve.setPos(-(now - self.startTime), 0)
                id += 1

        self.ptr5 += 1
        if self.ptr5 >= self.data5.shape[1]:
            tmp = self.data5
            self.data5 = np.empty((len(self.plots), self.data5.shape[1] * 2, 2), dtype=np.float32)
            print(self.data5.shape)
            self.data5[:, : tmp.shape[1]] = tmp

    def __del__(self):
        for _pw in self.pw:
            for k, pditem in self.plots.items():
                _pw.removeItem(pditem)


class ChartThread(QThread):
    sinOut = pyqtSignal(dict)

    def __init__(self, init_cb: Callable, get_data_cb: Callable, parent=None):
        super(ChartThread, self).__init__(parent)
        self.get_data_cb = get_data_cb
        self.isRun = True
        init_cb(get_data_cb)

    def __del__(self):
        print("del ChartThread")

    def stopRun(self):
        self.isRun = False

    def run(self):
        while self.isRun:
            res = self.get_data_cb()
            self.sinOut.emit(res)
        print(f"ChartThread run end")

def scan_correct_port(ports):
    _ok = [False, False]
    res={}

    for p in ports:
        com = p['device']
        if not _ok[0]:
            _ser = serial.Serial(com, 115200, timeout=0.5)
            _bus = simbatouch.RS485(_ser, 1)
            _bus.connect()
            try:
                _d = simbatouch.SBT906D(_bus)
                time.sleep(0.1)
                res['sensor'] = com
                _ok[0] = True
                continue
            except:
                _bus._lock.release()
                pass
            finally:
                _bus.disconnect()

        if not _ok[1]:
            _ser = serial.Serial(com, 115200, timeout=0.5)
            _bus = demo_driver.RS485(_ser, 2)
            _bus.connect()
            try:
                _d = demo_driver.LeadScrew(_bus, 1)
                time.sleep(0.1)
                res['motor'] = com
                _ok[1] = True
                continue
            except:
                _bus._lock.release()
                pass
            finally:
                _bus.disconnect()
        
    return res

# import pyqtgraph.examples
# pyqtgraph.examples.run()


# manager = Manager()
# shared_namespace = manager.Namespace()


if __name__ == "__main__":
    fs = ForceSensor.FSensors()
    print("UI start")
    myapp = QApplication(sys.argv)
    win = MainWin(fs)
    win.show()
    sys.exit(myapp.exec_())

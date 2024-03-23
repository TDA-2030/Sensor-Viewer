import sys
from typing import Callable, Dict, List
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QMainWindow, QMessageBox
from PyQt5.QtGui import QIcon, QPixmap, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QWaitCondition, QMutex, QTimer

import pyqtgraph as pg
import pyqtgraph.exporters
import seaborn as sns
import numpy as np
import time, traceback
from pathlib import Path
from multiprocessing import  Process, Manager

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
PRINT = print
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH


from sensor import wit_normal
import mainwin
from sensor import ForceSensor


class MainWin(QMainWindow):

    def __init__(self, _sensor:ForceSensor.FSensors, parent=None):
        super(QMainWindow, self).__init__(parent)
        self.ui = mainwin.Ui_MainWindow()
        self.ui.setupUi(self)

        self.fsensors = _sensor
        self.imu = wit_normal.WIT_IMU()


        self.ui.lineEdit_com.setText("/dev/ttyUSB0")
        self.ui.pushButton_start.clicked.connect(self._solt_start)
        self.ui.pushButton_fbgconfig.clicked.connect(self._solt_fbgconfig)
        self.ui.pushButton_fbga_config.clicked.connect(self._solt_fbgaconfig)
        self.ui.pushButton_sgconfig.clicked.connect(self._solt_sgconfig)
        self.ui.pushButton_atigconfig.clicked.connect(self._solt_aticonfig)
        self.ui.pushButton_mark.clicked.connect(self._solt_mark)
        self.marked:int = 0

        pg.setConfigOptions(leftButtonPan=True, antialias=True) 
        pg.setConfigOption('background', (30, 30, 33))
        pg.setConfigOption('foreground', 'w')

        self.ui.horizontalLayout = QVBoxLayout(self.ui.groupBox_chart)
        self.ui.horizontalLayout.setObjectName("verticalLayout")
        self.ui.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.pg_win = pg.GraphicsLayoutWidget(self.ui.groupBox_chart, show=True)
        self.ui.horizontalLayout.addWidget(self.pg_win)
        self.pw = []
        pw:pg.PlotItem = self.pg_win.addPlot(title="Raw data")
        labelStyle = {'color': '#ffffff', 'font-size': '16pt'}
        pw.getAxis("left").setLabel('PhysicValue', units='pm', **labelStyle)
        self.pw.append(pw)
        self.pg_win.nextRow()
        pw = self.pg_win.addPlot(title="Force Value")
        pw.getAxis("left").setLabel('Force', units='N', **labelStyle)
        self.pw.append(pw)
        self.pw:List[pg.PlotItem]

        for pw in self.pw:
            pw.showGrid(x=True,y=True,alpha=0.4)
            pw.addLegend()

            left_axis = pw.getAxis("left")
            bottom_axis = pw.getAxis("bottom")
            # left_axis.enableAutoSIPrefix(False)
            font = QFont()
            font.setPixelSize(16)
            left_axis.tickFont = font
            bottom_axis.tickFont = font
            bottom_axis.setLabel('Time', units='sec', **labelStyle)


    def _solt_start(self):
        if self.ui.pushButton_start.text() == "开始":
            print("start")

            if self.ui.checkBox_fbg.isChecked():
                text = self.ui.ipaddress_edit.text().strip()
                text = text.split(":")
                try:
                    self.fsensors.fs_connect_fbg(text[0], int(text[1]))
                except:
                    traceback.print_exc()
                    QMessageBox.warning(self, u'Warning', u'无法连接解调仪，请检查连接是否正确', buttons=QMessageBox.Ok)
            if self.ui.checkBox_sg.isChecked():
                text = self.ui.lineEdit_com.text().strip()
                buad = self.ui.lineEdit_com_buad.text().strip()
                try:
                    self.fsensors.fs_connect_zn(text, buad)
                except:
                    traceback.print_exc()
                    QMessageBox.warning(self, u'Warning', u'无法通过 RS485 连接力传感器，请检查连接是否正确', buttons=QMessageBox.Ok)
            if self.ui.checkBox_ati.isChecked():
                try:
                    self.fsensors.fs_connect_ati("192.168.1.1")
                except:
                    traceback.print_exc()
                    QMessageBox.warning(self, u'Warning', u'无法连接ATI力传感器，请检查连接是否正确', buttons=QMessageBox.Ok)

            if self.ui.checkBox_fbga.isChecked():
                text = self.ui.lineEdit_fbga.text().strip()
                text = text.split(":")
                try:
                    self.fsensors.fs_connect_fbga(text[0], int(text[1]))
                except:
                    traceback.print_exc()
                    QMessageBox.warning(self, u'Warning', u'无法连接FBGA解调仪，请检查连接是否正确', buttons=QMessageBox.Ok)

            if self.ui.checkBox_imu.isChecked():
                text = self.ui.lineEdit_imu_port.text().strip()
                self.imu.connect(text, 9600)

            def _get_data() -> Dict[str, Dict[str, float]]:
                m = {"mark":{"m":self.marked}}
                _d = self.fsensors.fs_get_data()
                _d.update(m)
                if self.imu.is_connected:
                    _d = {**_d, **self.imu.get_imu_data()}
                return _d


            print("------------")
            # 读一次数据，判断有多少个通道
            _d = _get_data()
            id = 0
            self.plots = {}
            
            palette = sns.color_palette(None, 32)
            palette = (np.array(palette)*255).tolist()
            
            lines = {
                "imu": Qt.DashLine,
                "fbg": Qt.DotLine,
                # "fbg": Qt.SolidLine,
            }

            for (sensor,svalue) in _d.items():
                for (key,value) in svalue.items():
                    key:str
                    penstate_r = pg.mkPen(width=2, color=palette[id], style=lines.get(sensor, Qt.SolidLine))  # 画笔
                    if key.startswith("Force_"):
                        self.plots[sensor+key] = self.pw[1].plot(pen=penstate_r, name=sensor+"-"+key)
                    else:
                        self.plots[sensor+key] = self.pw[0].plot(pen=penstate_r, name=sensor+"-"+key)
                    id += 1
            if len(self.plots)==0: # 没有可用通道，直接退出
                print("no data to plot")
                return
            self.plots:Dict[str, pg.PlotDataItem]

            self.ui.pushButton_start.setText("关闭")
            if self.ui.checkBox_fbg.isChecked():
                self.ui.ipaddress_edit.setEnabled(False)
            if self.ui.checkBox_fbga.isChecked():
                self.ui.lineEdit_fbga.setEnabled(False)
            if self.ui.checkBox_sg.isChecked():
                self.ui.lineEdit_com.setEnabled(False)
                self.ui.lineEdit_com_buad.setEnabled(False)

            self.ui.checkBox_fbg.setEnabled(False)
            self.ui.checkBox_imu.setEnabled(False)
            self.ui.checkBox_sg.setEnabled(False)
            self.ui.checkBox_ati.setEnabled(False)
            self.ui.checkBox_fbga.setEnabled(False)

            # self.ui.tabWidget.tabBar().setEnabled(False)
            self.ui.pushButton_start.setIcon(QIcon(QPixmap(":/main/disconnect.png")))
            
            self.chartthread = ChartThread(_get_data, self)
            self.prl = PlotRollLine(self.pw, self.plots)
            self.chartthread.sinOut.connect(lambda res: self.prl.drawLine(res))
            self.chartthread.start()
            for pw in self.pw:
                pw.enableAutoRange(axis='y', enable=True)

        else:
            print("stop")
            self.ui.pushButton_start.setIcon(QIcon(QPixmap(":/main/connect.png")))
            if self.ui.checkBox_fbg.isChecked():
                self.ui.ipaddress_edit.setEnabled(True)
            if self.ui.checkBox_fbga.isChecked():
                self.ui.lineEdit_fbga.setEnabled(True)
            if self.ui.checkBox_sg.isChecked():
                self.ui.lineEdit_com.setEnabled(True)
                self.ui.lineEdit_com_buad.setEnabled(True)
            if self.ui.checkBox_ati.isChecked():
                pass
            if self.ui.checkBox_imu.isChecked():
                self.imu.disconnect()

            self.ui.checkBox_fbg.setEnabled(True)
            self.ui.checkBox_imu.setEnabled(True)
            self.ui.checkBox_sg.setEnabled(True)
            self.ui.checkBox_ati.setEnabled(True)
            self.ui.checkBox_fbga.setEnabled(True)

            self.fsensors.fs_disconnect()
            self.ui.pushButton_start.setText("开始")
            # self.ui.tabWidget.tabBar().setEnabled(True)
            self.chartthread.stopRun()

            if self.ui.checkBox_autosave.isChecked():
                for i, pw in enumerate(self.pw):
                    ex = pyqtgraph.exporters.CSVExporter(pw)
                    fname = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime()) + f"-plot{i}.csv"
                    ex.export(fname)

    def _solt_mark(self):
        if self.marked == 0:
            self.marked = 1
            self.ui.pushButton_mark.setText("标记结束")
        else:
            self.marked = 0
            self.ui.pushButton_mark.setText("标记开始")

    def _solt_fbgconfig(self):
        self.fsensors.fbg.tare()

    def _solt_fbgaconfig(self):
        self.fsensors.fbga.FBGA_tare()

    def _solt_sgconfig(self):
        self.fsensors.znbsq.tare()

    def _solt_aticonfig(self):
        self.fsensors.ati.tare()


class PlotRollLine():
    def __init__(self, pw:List[pg.PlotWidget], plots:Dict[str, pg.PlotDataItem]) -> None:
        self.startTime = time.perf_counter()
        self.plots = plots
        self.ptr5 = 0
        self.data5 = np.empty((len(plots),256,2), dtype=np.float32)
        self.pw = pw
        for pw in self.pw:
            pw.setXRange(-5, 0)

    def drawLine(self, data):
        id = 0
        for (sensor,svalue) in data.items():
            for (key,value) in svalue.items():
                curve:pg.PlotDataItem = self.plots[sensor+key]
                now = time.perf_counter()
                self.data5[id, self.ptr5,0] = now - self.startTime
                self.data5[id, self.ptr5,1] = value # random.random()*100
                curve.setData(x=self.data5[id, :self.ptr5, 0], y=self.data5[id, :self.ptr5, 1])
                curve.setPos(-(now-self.startTime), 0)
                id += 1

        self.ptr5 += 1
        if self.ptr5 >= self.data5.shape[1]:
            tmp = self.data5
            self.data5 = np.empty((len(self.plots), self.data5.shape[1] * 2, 2), dtype=np.float32)
            print(self.data5.shape)
            self.data5[:, :tmp.shape[1]] = tmp

    def __del__(self):
        for _pw in self.pw:
            for k, pditem in self.plots.items():
                _pw.removeItem(pditem)


class ChartThread(QThread):
    sinOut = pyqtSignal(dict)

    def __init__(self, get_data_cb:Callable, parent=None):
        super(ChartThread, self).__init__(parent)
        self.get_data_cb = get_data_cb
        self.last_t = 0
        self.elapse = 30
        self.isRun = True
        self.mutex = QMutex()
        self.cond = QWaitCondition()
        

    def __del__(self):
        self.wait()

    def stopRun(self):
        self.isRun = False

    def get_real_fps(self) -> float:
        return 1.0 / self.elapse

    def run(self):
        while self.isRun:
            t0 = time.time()
            res = self.get_data_cb()
            self.sinOut.emit(res)
            t = time.time()
            self.elapse = t - self.last_t
            self.msleep(5)

# import pyqtgraph.examples
# pyqtgraph.examples.run()



# manager = Manager()
# shared_namespace = manager.Namespace()

def UI_Run(sensor):
    print("UI start")
    myapp = QApplication(sys.argv)
    win = MainWin(sensor)
    win.show()
    sys.exit(myapp.exec_())

def UI_Start(sensor)->Process:
    # shared_namespace.my_object = bot
    # p = Process(target=UI_Run,args=(bot,)) #实例化进程对象
    # p.start()
    # return p
    UI_Run(sensor)

if __name__ == '__main__':
    fs = ForceSensor.FSensors()
    p = UI_Start(fs)
    # p.join()

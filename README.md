# Sensor Viewer

一个采集多种传感器数据并实时显示的工具。

<img src="./view.png" style="zoom:60%;" />

**支持的传感器：**

- [x] FBG 解调仪
- [x] FBGA 解调仪
- [x] 中诺一维力传感器
- [x] ATI 六维力传感器
- [x] IMU姿态传感器

## 快速使用

1. 从 [Release](https://github.com/TDA-2030/Sensor-Viewer/releases) 下载最新程序
2. 解压缩后运行`run_sample.bat`打开采样程序；或运行`run_calibrate.bat`打开自动标定程序

## 从源码使用

Python版本： python3.8.10

1. 克隆代码：
    ```shell
    git clone https://github.com/TDA-2030/Sensor-Viewer.git
    ```

2. python 包安装：

   ```shell
   pip install -r requirements.txt
   ```

3. 安装qt开发工具：(可选)
    ```shell
    pip install pyqt5-tools
    ```

4. 运行`python ui.py` 打开界面

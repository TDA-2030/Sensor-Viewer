# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwin_ac.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1069, 668)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/main/logo.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setStyleSheet("#groupBox{\n"
"    \n"
"    background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:1, y2:0.471, stop:0.0225989 rgba(130, 155, 164, 255), stop:0.494318 rgba(71, 166, 182, 255), stop:0.931818 rgba(64, 161, 191, 255));\n"
"border: 0px solid #42adff;\n"
"\n"
"}\n"
"\n"
"QPushButton{\n"
"    height: 22px;\n"
"    border-radius: 8px;\n"
"    border: 0px solid;\n"
"    \n"
"    border-color: rgb(29, 211, 176);\n"
"    \n"
"    background-color: qlineargradient(spread:pad, x1:0.227, y1:0.46, x2:0.886, y2:0.193182, stop:0 rgba(191, 189, 255, 255), stop:0.596591 rgba(131, 202, 235, 255), stop:1 rgba(159, 214, 208, 255));\n"
"    color: rgb(255, 255, 255);\n"
"    font: 15px \"SimHei\"\n"
"}\n"
"QPushButton:hover{\n"
"    background-color: rgba(176, 208, 211, 160);\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: rgba(176, 208, 211, 230);\n"
"}")
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.gridLayout = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName("gridLayout")
        self.groupBox_3 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_3.setMaximumSize(QtCore.QSize(16777215, 60))
        self.groupBox_3.setStyleSheet("QGroupBox{\n"
"\n"
"border:0px solid rgb(10, 100, 10);\n"
"border-bottom:1px solid rgb(100, 100, 10);\n"
"}")
        self.groupBox_3.setTitle("")
        self.groupBox_3.setObjectName("groupBox_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.groupBox_3)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setMinimumSize(QtCore.QSize(150, 50))
        self.label_2.setMaximumSize(QtCore.QSize(360, 80))
        self.label_2.setStyleSheet("color: rgb(255, 244, 244);\n"
"font: 75 25pt \"Roman times\";\n"
"font-weight: bold;\n"
"border-image:url(:/main/logo_transparent_ac.png);")
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.gridLayout.addWidget(self.groupBox_3, 0, 0, 1, 2)
        self.groupBox_2 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_2.setMaximumSize(QtCore.QSize(240, 16777215))
        self.groupBox_2.setStyleSheet("#groupBox_2{\n"
"border-left:1px solid rgb(100, 100, 10);\n"
"}\n"
"QGroupBox{\n"
"border:0px solid rgb(10, 100, 10);\n"
"\n"
"}\n"
"QLineEdit{\n"
"background-color: rgba(255, 170, 255, 50);\n"
"color: rgb(255, 255, 255);\n"
"border:0px solid rgb(10, 100, 10);\n"
"border-left:5px solid rgb(10, 100, 10);\n"
"}\n"
"QComboBox{\n"
"background-color: rgba(255, 170, 255, 50);\n"
"color: rgb(255, 255, 255);\n"
"}")
        self.groupBox_2.setTitle("")
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.verticalLayout.setContentsMargins(4, 4, 4, 4)
        self.verticalLayout.setSpacing(16)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tabWidget = QtWidgets.QTabWidget(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tabWidget.setStyleSheet("\n"
"\n"
"QTabWidget::pane {\n"
"border-top:1px solid rgba(200, 200, 100, 150);\n"
"\n"
"background: rgba(150, 100, 100, 0); \n"
"}\n"
"\n"
"\n"
"QTabBar::tab\n"
"{\n"
"    \n"
"    color: rgb(241, 241, 241);\n"
"     font: 12pt \"SimHei\";\n"
"     background-color:rgba(104,191,249,50);\n"
"     min-width: 40px;\n"
"     min-height: 30px;\n"
"     padding:1px;\n"
"\n"
"     border-top-left-radius: 8px;\n"
"     border-top-right-radius: 8px;\n"
"  \n"
"}\n"
"\n"
"QTabBar::tab:!selected\n"
"{\n"
"    margin-top:5px;\n"
"}\n"
"\n"
"QTabBar::tab:selected\n"
"{\n"
"    font: 13pt \"SimHei\";\n"
"    background-color: rgba(228, 233, 242, 50);\n"
"    \n"
"    border-top:1px solid rgb(100, 100, 10);\n"
"    border-left:1px solid rgb(100, 100, 10);\n"
"    border-right:1px solid rgb(100, 100, 10);\n"
"\n"
"}\n"
"\n"
"QLineEdit{\n"
"background-color: rgba(255, 170, 255, 50);\n"
"color: rgb(255, 255, 255);\n"
"border:0px solid rgb(10, 100, 10);\n"
"border-left:5px solid rgb(10, 100, 10);\n"
"}\n"
"")
        self.tabWidget.setTabPosition(QtWidgets.QTabWidget.North)
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_fbg = QtWidgets.QWidget()
        self.tab_fbg.setStyleSheet("#tab_fbg{background-color: rgba(255, 255, 255, 0);}")
        self.tab_fbg.setObjectName("tab_fbg")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.tab_fbg)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.ipaddress_edit = QtWidgets.QLineEdit(self.tab_fbg)
        self.ipaddress_edit.setStyleSheet("")
        self.ipaddress_edit.setObjectName("ipaddress_edit")
        self.verticalLayout_3.addWidget(self.ipaddress_edit)
        self.pushButton_fbgconfig = QtWidgets.QPushButton(self.tab_fbg)
        self.pushButton_fbgconfig.setObjectName("pushButton_fbgconfig")
        self.verticalLayout_3.addWidget(self.pushButton_fbgconfig)
        self.checkBox_fbg = QtWidgets.QCheckBox(self.tab_fbg)
        self.checkBox_fbg.setStyleSheet("color: rgb(255, 255, 255);")
        self.checkBox_fbg.setObjectName("checkBox_fbg")
        self.verticalLayout_3.addWidget(self.checkBox_fbg)
        self.tabWidget.addTab(self.tab_fbg, "")
        self.tab_fbga = QtWidgets.QWidget()
        self.tab_fbga.setStyleSheet("#tab_fbga{background-color: rgba(255, 255, 255, 0);}")
        self.tab_fbga.setObjectName("tab_fbga")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.tab_fbga)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.lineEdit_fbga = QtWidgets.QLineEdit(self.tab_fbga)
        self.lineEdit_fbga.setObjectName("lineEdit_fbga")
        self.verticalLayout_8.addWidget(self.lineEdit_fbga)
        self.pushButton_fbga_config = QtWidgets.QPushButton(self.tab_fbga)
        self.pushButton_fbga_config.setObjectName("pushButton_fbga_config")
        self.verticalLayout_8.addWidget(self.pushButton_fbga_config)
        self.checkBox_fbga = QtWidgets.QCheckBox(self.tab_fbga)
        self.checkBox_fbga.setStyleSheet("color: rgb(255, 255, 255);")
        self.checkBox_fbga.setObjectName("checkBox_fbga")
        self.verticalLayout_8.addWidget(self.checkBox_fbga)
        self.tabWidget.addTab(self.tab_fbga, "")
        self.verticalLayout.addWidget(self.tabWidget)
        self.groupBox_10 = QtWidgets.QGroupBox(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_10.sizePolicy().hasHeightForWidth())
        self.groupBox_10.setSizePolicy(sizePolicy)
        self.groupBox_10.setTitle("")
        self.groupBox_10.setObjectName("groupBox_10")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.groupBox_10)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.groupBox_10)
        self.label.setStyleSheet("color: rgb(255, 255, 255);")
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.comboBox_mot_port = QtWidgets.QComboBox(self.groupBox_10)
        self.comboBox_mot_port.setObjectName("comboBox_mot_port")
        self.horizontalLayout.addWidget(self.comboBox_mot_port)
        self.verticalLayout_6.addLayout(self.horizontalLayout)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_3 = QtWidgets.QLabel(self.groupBox_10)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_3.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.comboBox_sen_port = QtWidgets.QComboBox(self.groupBox_10)
        self.comboBox_sen_port.setObjectName("comboBox_sen_port")
        self.horizontalLayout_3.addWidget(self.comboBox_sen_port)
        self.verticalLayout_6.addLayout(self.horizontalLayout_3)
        self.pushButton_connect = QtWidgets.QPushButton(self.groupBox_10)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/main/connect.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_connect.setIcon(icon1)
        self.pushButton_connect.setObjectName("pushButton_connect")
        self.verticalLayout_6.addWidget(self.pushButton_connect)
        self.checkBox_autosave = QtWidgets.QCheckBox(self.groupBox_10)
        self.checkBox_autosave.setStyleSheet("color: rgb(255, 255, 255);")
        self.checkBox_autosave.setChecked(True)
        self.checkBox_autosave.setObjectName("checkBox_autosave")
        self.verticalLayout_6.addWidget(self.checkBox_autosave)
        self.verticalLayout.addWidget(self.groupBox_10)
        self.scrollArea = QtWidgets.QScrollArea(self.groupBox_2)
        self.scrollArea.setStyleSheet("QScrollArea{\n"
"    background: transparent;\n"
"}\n"
"QWidget#scrollAreaWidgetContents{\n"
"    background: transparent;\n"
"}\n"
"\n"
"QPushButton{\n"
"    height: 22px;\n"
"    border-radius: 8px;\n"
"    background-color: rgb(85, 170, 255);\n"
"    color: rgb(255, 255, 255);\n"
"    font:15px \"SimSun\";\n"
"}\n"
"QPushButton:hover{\n"
"    background-color: rgb(85, 85, 255);\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: rgb(0, 0, 255);\n"
"}\n"
"\n"
"\n"
"\n"
"QSlider::groove:horizontal{ \n"
"    height: 16px; \n"
"    left: 0px; \n"
"    right: 0px; \n"
"    border:0px; \n"
"    border-radius:6px;   \n"
"    background:rgba(0,0,0,50);\n"
" } \n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"    background-color: rgb(161, 199, 255);\n"
"\n"
"}\n"
"\n"
" QSlider::handle:horizontal{ \n"
"    width:  16px; \n"
"    border-radius:6px;  \n"
"    \n"
"    background-color: rgb(0, 170, 255);\n"
"        \n"
"} \n"
"")
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 229, 344))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.scrollAreaWidgetContents)
        self.gridLayout_5.setContentsMargins(0, 6, 0, 6)
        self.gridLayout_5.setSpacing(0)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.groupBox_8 = QtWidgets.QGroupBox(self.scrollAreaWidgetContents)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_8.sizePolicy().hasHeightForWidth())
        self.groupBox_8.setSizePolicy(sizePolicy)
        self.groupBox_8.setObjectName("groupBox_8")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.groupBox_8)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.pushButton_cal_lm = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_cal_lm.setObjectName("pushButton_cal_lm")
        self.gridLayout_7.addWidget(self.pushButton_cal_lm, 4, 0, 1, 1)
        self.pushButton_cal_rh = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_cal_rh.setObjectName("pushButton_cal_rh")
        self.gridLayout_7.addWidget(self.pushButton_cal_rh, 5, 2, 1, 1)
        self.pushButton_cal_lp = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_cal_lp.setObjectName("pushButton_cal_lp")
        self.gridLayout_7.addWidget(self.pushButton_cal_lp, 4, 1, 1, 1)
        self.pushButton_cal_lh = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_cal_lh.setCheckable(True)
        self.pushButton_cal_lh.setObjectName("pushButton_cal_lh")
        self.gridLayout_7.addWidget(self.pushButton_cal_lh, 4, 2, 1, 1)
        self.pushButton_cal_rm = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_cal_rm.setObjectName("pushButton_cal_rm")
        self.gridLayout_7.addWidget(self.pushButton_cal_rm, 5, 0, 1, 1)
        self.pushButton_cal_rp = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_cal_rp.setObjectName("pushButton_cal_rp")
        self.gridLayout_7.addWidget(self.pushButton_cal_rp, 5, 1, 1, 1)
        self.label_cal_set_speed = QtWidgets.QLabel(self.groupBox_8)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_cal_set_speed.sizePolicy().hasHeightForWidth())
        self.label_cal_set_speed.setSizePolicy(sizePolicy)
        self.label_cal_set_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.label_cal_set_speed.setObjectName("label_cal_set_speed")
        self.gridLayout_7.addWidget(self.label_cal_set_speed, 3, 0, 1, 1)
        self.horizontalSlider_speed = QtWidgets.QSlider(self.groupBox_8)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_speed.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_speed.setSizePolicy(sizePolicy)
        self.horizontalSlider_speed.setMinimum(1)
        self.horizontalSlider_speed.setMaximum(100)
        self.horizontalSlider_speed.setPageStep(1)
        self.horizontalSlider_speed.setProperty("value", 50)
        self.horizontalSlider_speed.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_speed.setObjectName("horizontalSlider_speed")
        self.gridLayout_7.addWidget(self.horizontalSlider_speed, 3, 1, 1, 2)
        self.gridLayout_5.addWidget(self.groupBox_8, 0, 0, 1, 1)
        self.groupBox_9 = QtWidgets.QGroupBox(self.scrollAreaWidgetContents)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_9.sizePolicy().hasHeightForWidth())
        self.groupBox_9.setSizePolicy(sizePolicy)
        self.groupBox_9.setObjectName("groupBox_9")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.groupBox_9)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.pushButton_cal_start = QtWidgets.QPushButton(self.groupBox_9)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_cal_start.sizePolicy().hasHeightForWidth())
        self.pushButton_cal_start.setSizePolicy(sizePolicy)
        self.pushButton_cal_start.setMaximumSize(QtCore.QSize(100, 100))
        self.pushButton_cal_start.setStyleSheet("QPushButton{\n"
"    border-radius: 10px;\n"
"}")
        self.pushButton_cal_start.setObjectName("pushButton_cal_start")
        self.gridLayout_6.addWidget(self.pushButton_cal_start, 0, 3, 1, 1)
        self.gridLayout_5.addWidget(self.groupBox_9, 0, 1, 1, 1)
        self.groupBox_5 = QtWidgets.QGroupBox(self.scrollAreaWidgetContents)
        self.groupBox_5.setStyleSheet("")
        self.groupBox_5.setObjectName("groupBox_5")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox_5)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.groupBox_6 = QtWidgets.QGroupBox(self.groupBox_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_6.sizePolicy().hasHeightForWidth())
        self.groupBox_6.setSizePolicy(sizePolicy)
        self.groupBox_6.setStyleSheet("QGroupBox{\n"
"border: opx solid;\n"
"}")
        self.groupBox_6.setObjectName("groupBox_6")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.groupBox_6)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_curve_2 = QtWidgets.QLabel(self.groupBox_6)
        self.label_curve_2.setObjectName("label_curve_2")
        self.horizontalLayout_4.addWidget(self.label_curve_2)
        self.doubleSpinBox_start_force = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.doubleSpinBox_start_force.setDecimals(1)
        self.doubleSpinBox_start_force.setMinimum(0.0)
        self.doubleSpinBox_start_force.setMaximum(1000.0)
        self.doubleSpinBox_start_force.setSingleStep(5.0)
        self.doubleSpinBox_start_force.setProperty("value", 0.0)
        self.doubleSpinBox_start_force.setObjectName("doubleSpinBox_start_force")
        self.horizontalLayout_4.addWidget(self.doubleSpinBox_start_force)
        self.verticalLayout_4.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_curve_3 = QtWidgets.QLabel(self.groupBox_6)
        self.label_curve_3.setObjectName("label_curve_3")
        self.horizontalLayout_5.addWidget(self.label_curve_3)
        self.doubleSpinBox_end_force = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.doubleSpinBox_end_force.setDecimals(1)
        self.doubleSpinBox_end_force.setMinimum(10.0)
        self.doubleSpinBox_end_force.setMaximum(1000.0)
        self.doubleSpinBox_end_force.setSingleStep(5.0)
        self.doubleSpinBox_end_force.setProperty("value", 200.0)
        self.doubleSpinBox_end_force.setObjectName("doubleSpinBox_end_force")
        self.horizontalLayout_5.addWidget(self.doubleSpinBox_end_force)
        self.verticalLayout_4.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_curve_4 = QtWidgets.QLabel(self.groupBox_6)
        self.label_curve_4.setObjectName("label_curve_4")
        self.horizontalLayout_6.addWidget(self.label_curve_4)
        self.spinBox_point_num = QtWidgets.QSpinBox(self.groupBox_6)
        self.spinBox_point_num.setMinimum(2)
        self.spinBox_point_num.setMaximum(1000)
        self.spinBox_point_num.setProperty("value", 11)
        self.spinBox_point_num.setObjectName("spinBox_point_num")
        self.horizontalLayout_6.addWidget(self.spinBox_point_num)
        self.verticalLayout_4.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_curve_5 = QtWidgets.QLabel(self.groupBox_6)
        self.label_curve_5.setObjectName("label_curve_5")
        self.horizontalLayout_7.addWidget(self.label_curve_5)
        self.spinBox_repeat = QtWidgets.QSpinBox(self.groupBox_6)
        self.spinBox_repeat.setMinimum(1)
        self.spinBox_repeat.setProperty("value", 1)
        self.spinBox_repeat.setObjectName("spinBox_repeat")
        self.horizontalLayout_7.addWidget(self.spinBox_repeat)
        self.verticalLayout_4.addLayout(self.horizontalLayout_7)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_curve_6 = QtWidgets.QLabel(self.groupBox_6)
        self.label_curve_6.setObjectName("label_curve_6")
        self.gridLayout_3.addWidget(self.label_curve_6, 0, 0, 1, 1)
        self.doubleSpinBox_interval = QtWidgets.QDoubleSpinBox(self.groupBox_6)
        self.doubleSpinBox_interval.setDecimals(1)
        self.doubleSpinBox_interval.setMinimum(1.0)
        self.doubleSpinBox_interval.setMaximum(18000.0)
        self.doubleSpinBox_interval.setSingleStep(0.5)
        self.doubleSpinBox_interval.setProperty("value", 3.0)
        self.doubleSpinBox_interval.setObjectName("doubleSpinBox_interval")
        self.gridLayout_3.addWidget(self.doubleSpinBox_interval, 0, 2, 1, 1)
        self.comboBox_cal_mode = QtWidgets.QComboBox(self.groupBox_6)
        self.comboBox_cal_mode.setObjectName("comboBox_cal_mode")
        self.gridLayout_3.addWidget(self.comboBox_cal_mode, 1, 2, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.groupBox_6)
        self.label_4.setObjectName("label_4")
        self.gridLayout_3.addWidget(self.label_4, 1, 0, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_3)
        self.gridLayout_2.addWidget(self.groupBox_6, 0, 0, 1, 1)
        self.gridLayout_5.addWidget(self.groupBox_5, 1, 0, 1, 2)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout.addWidget(self.scrollArea)
        self.gridLayout.addWidget(self.groupBox_2, 1, 1, 1, 1)
        self.groupBox_chart = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_chart.setStyleSheet("QGroupBox{\n"
"border:0px solid rgb(10, 100, 10);\n"
"\n"
"}")
        self.groupBox_chart.setTitle("")
        self.groupBox_chart.setObjectName("groupBox_chart")
        self.gridLayout.addWidget(self.groupBox_chart, 1, 0, 1, 1)
        self.verticalLayout_2.addWidget(self.groupBox)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Sensor Viewer"))
        self.ipaddress_edit.setText(_translate("MainWindow", "192.168.1.101:7102"))
        self.pushButton_fbgconfig.setText(_translate("MainWindow", "清零基线"))
        self.checkBox_fbg.setText(_translate("MainWindow", "采集"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_fbg), _translate("MainWindow", "FBG"))
        self.lineEdit_fbga.setText(_translate("MainWindow", "192.168.0.111:4567"))
        self.pushButton_fbga_config.setText(_translate("MainWindow", "清零基线"))
        self.checkBox_fbga.setText(_translate("MainWindow", "采集"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_fbga), _translate("MainWindow", "FBGA"))
        self.label.setText(_translate("MainWindow", "电机端口："))
        self.label_3.setText(_translate("MainWindow", "传感器端口："))
        self.pushButton_connect.setText(_translate("MainWindow", "连接"))
        self.checkBox_autosave.setText(_translate("MainWindow", "自动保存"))
        self.groupBox_8.setTitle(_translate("MainWindow", "推杆"))
        self.pushButton_cal_lm.setText(_translate("MainWindow", "<"))
        self.pushButton_cal_rh.setText(_translate("MainWindow", "归位"))
        self.pushButton_cal_lp.setText(_translate("MainWindow", ">"))
        self.pushButton_cal_lh.setText(_translate("MainWindow", "归位"))
        self.pushButton_cal_rm.setText(_translate("MainWindow", "<"))
        self.pushButton_cal_rp.setText(_translate("MainWindow", ">"))
        self.label_cal_set_speed.setText(_translate("MainWindow", "速度()"))
        self.groupBox_9.setTitle(_translate("MainWindow", "动作"))
        self.pushButton_cal_start.setText(_translate("MainWindow", "开始"))
        self.groupBox_5.setTitle(_translate("MainWindow", "参数"))
        self.label_curve_2.setText(_translate("MainWindow", "起始力(g)"))
        self.label_curve_3.setText(_translate("MainWindow", "终止力(g)"))
        self.label_curve_4.setText(_translate("MainWindow", "点数"))
        self.label_curve_5.setText(_translate("MainWindow", "重复"))
        self.label_curve_6.setText(_translate("MainWindow", "间隔时间(s)"))
        self.label_4.setText(_translate("MainWindow", "模式"))
import main_ac_rc

pyinstaller ^
--add-data "D:/work/HNU-learning/acupuncture-robot/src/force_sensor/scripts/FBG.py;." ^
--add-data "D:/work/HNU-learning/acupuncture-robot/src/force_sensor/scripts/ForceSensor.py;." ^
--add-data "D:/work/HNU-learning/acupuncture-robot/src/force_sensor/scripts/ZNBSQ.py;." ^
--add-data "D:/work/HNU-learning/acupuncture-robot/src/force_sensor/scripts/NetFT.py;." ^
--add-data "D:/work/HNU-learning/acupuncture-robot/src/wit_ros_imu/demo/wit_normal.py;." ^
--hidden-import pyserial ^
-w -i ./logo.ico ui.py

rd /q /s build
del /q ui.spec
move dist\ui.exe ui.exe
rd /q dist

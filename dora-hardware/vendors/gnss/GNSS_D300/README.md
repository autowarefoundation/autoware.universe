# 传感器驱动

基于 Dora0.3.2环境开发的GNSS定位。



# 文件代码说明

1. gnss/gnss.py: 定位驱动代码；
2. gnss/Nmea_utils.py: 协议Nmea的格式代码，用于解析Nmea协议；
3. gnss/DoraNmeaDriver_utils.py: Dora框架下的Nmea存储格式代码，用于存储定位数据；
4. gnss/gnss_D300_driver_dora.py: 用于打印GNSS的定位数据；
5. Gnss_to_ros2.yml： 数据流脚本。

## 用法

```
dora up
sudo chmod 777 /dev/ttyUSB0 
sudo chmod 777 /dev/ttyUSB1
dora start Gnss_to_ros2.yml --name test
```
目前IMU对应/dev/ttyUSB0，GNSS对应/dev/ttyUSB1；

Gnss驱动代码：gnss_D300_driver_dora.py;

先插的传感器是/dev/ttyUSB0 ，后插的传感器是/dev/ttyUSB1。

## 日志

```bash
dora logs test gnss_sub #查看gnss的数据
```

RVIZ2可视化IMU

```bash
sudo apt-get install ros-galactic-imu-tools
```


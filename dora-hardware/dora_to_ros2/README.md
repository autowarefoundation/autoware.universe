# 传感器驱动

基于 Dora0.3.2环境开发的100D4陀螺仪和GNSS定位。

## 文件清单

**gnss**  存放gnss定位驱动代码

**imu**  存放imu角度传感器驱动代码

**lidar**  存放lidar激光雷达驱动代码
**Imu_Gnss_to_ros2.yml**   Dora数据流文件,将Imu和Gnss传感器的数据发送到ros2中

## 背景

将IMU驱动与GNSS驱动放入一个数据流当中，目前IMU可以直接发到ros2进行显示，但GNSS可能是室内定位的原因，定位数据都是无效定位。

# 文件代码说明

1. imu/imu.py: 陀螺仪驱动代码；
2. gnss/gnss.py: 定位驱动代码；
3. gnss/Nmea_utils.py: 协议Nmea的格式代码，用于解析Nmea协议；
4. gnss/DoraNmeaDriver_utils.py: Dora框架下的Nmea存储格式代码，用于存储定位数据；
5. gnss/gnss_D300_driver_dora.py: 用于打印GNSS的定位数据；
6. dataflowImuGnss.yml： 数据流脚本。

## 用法

```
PATH=$PATH:$(pwd)
cd /home/crp/dora_project/dora-rs/dora-hardware/vendors/autoware/
dora up
sudo chmod 777 /dev/ttyUSB0 
sudo chmod 777 /dev/ttyUSB1
dora start Imu_Gnss_to_ros2.yml --name test
```
目前IMU对应/dev/ttyUSB0，GNSS对应/dev/ttyUSB1；

Imu驱动代码：imu.py

Gnss驱动代码：gnss_D300_driver_dora.py;

先插的传感器是/dev/ttyUSB0 ，后插的传感器是/dev/ttyUSB1。

## 日志

```bash
dora logs test imu #查看imu的数据
dora logs test gnss_sub #查看gnss的数据
```

RVIZ2可视化IMU

```bash
sudo apt-get install ros-galactic-imu-tools
```


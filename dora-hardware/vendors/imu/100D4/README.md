# 传感器驱动

基于 Dora0.3.2环境开发的100D4陀螺仪。

## 背景

将IMU驱动与GNSS驱动放入一个数据流当中，目前IMU可以直接发到ros2进行显示。

# 文件代码说明

1. imu/imu.py: 陀螺仪驱动代码；
6. Imu_to_ros2.yml： 数据流脚本。

## 用法

```
dora up
sudo chmod 777 /dev/ttyUSB0 
sudo chmod 777 /dev/ttyUSB1
dora start Imu_to_ros2.yml --name test
```
目前IMU对应/dev/ttyUSB0，GNSS对应/dev/ttyUSB1；

Imu驱动代码：imu.py

先插的传感器是/dev/ttyUSB0 ，后插的传感器是/dev/ttyUSB1。

## 日志

```bash
dora logs test imu #查看imu的数据
```

RVIZ2可视化IMU

```bash
sudo apt-get install ros-galactic-imu-tools
```


# 10 如何使用坐标变换功能



## 10.1 简介

rslidar_sdk支持对点云进行坐标变换，本文档展示如何作这种变换。 

在阅读本文档之前，请确保已阅读雷达用户手册和[隐藏参数介绍](../intro/03_hiding_parameters_intro_CN.md)。



## 10.2 依赖库

rslidar_sdk的坐标变换基于libeigen库，所以要先安装它。

```bash
sudo apt-get install libeigen3-dev
```



## 10.3 编译

要启用坐标变换，编译rslidar_sdk时，需要将```ENABLE_TRANSFORM```选项设置为```ON```.

- 直接编译

  ```bash
  cmake -DENABLE_TRANSFORM=ON ..
  ```

- ROS

  ```bash
  catkin_make -DENABLE_TRANSFORM=ON
  ```

- ROS2

  ```bash
  colcon build --cmake-args '-DENABLE_TRANSFORM=ON'
  ```



## 10.4 设置雷达参数

在`config.yaml`中，设置`lidar-lidar`部分的参数`x`、, `y`、 `z`、 `roll`、 `pitch` 、`yaw`。

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
lidar:
  - driver:
      lidar_type: RS128            
      msop_port: 6699              
      difop_port: 7788             
      start_angle: 0               
      end_angle: 360             
      min_distance: 0.2            
      max_distance: 200           
      use_lidar_clock: false       
      pcap_path: /home/robosense/lidar.pcap     
      x: 1
      y: 0
      z: 2.5
      roll: 0.1
      pitch: 0.2
      yaw: 1.57
```



## 10.5 运行

运行程序。

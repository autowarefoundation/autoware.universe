# 10 How to use coordinate transformation



## 10.1 Introduction

rslidar_sdk can transform the coordinate of point cloud. This document illustrate how to do so.

Please check the  [Intro to hiding parameters](../intro/03_hiding_parameters_intro.md) for more details. Here is an example of the config.yaml.



## 10.2 Dependencies

rslidar_sdk depends on the libeigen library to do coordinate transformation. Please install it first.

```bash
sudo apt-get install libeigen3-dev
```



## 10.3 Compile

To enable transformation, set the CMake option ```ENABLE_TRANSFORM```to be ```ON```.

- Compile directly

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



## 10.4 Set LiDAR parameters

In the `lidar-driver` part of `config.yaml`, set the hiding parameter`x`, `y`, `z`, `roll`, `pitch` ,`yaw`. 

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



## 10.5 Run

Run the program.

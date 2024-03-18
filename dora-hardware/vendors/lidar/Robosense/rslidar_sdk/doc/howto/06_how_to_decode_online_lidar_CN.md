# 6 如何连接在线雷达



## 6.1 简介

本文档描述如何连接在线雷达，并发送点云数据到ROS。

在阅读本文档之前， 请确保已经阅读过雷达用户手册和[参数简介](../intro/02_parameter_intro_CN.md) 。



## 6.2 步骤

### 6.2.1 获取数据端口号

根据雷达用户手册连接雷达, 并设置好您的电脑的IP地址。

请参考雷达用户手册，或使用第三方工具（如WireShark等）得到雷达的MSOP端口号和DIFOP端口号。端口的默认值分别为```6699```和```7788```。 

### 6.2.2 设置参数文件

设置参数文件```config.yaml```。

#### 6.2.2.1 common部分

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

消息来源于在线雷达，因此请设置```msg_source=1```。

将点云发送到ROS以便查看，因此设置 ```send_point_cloud_ros = true``` 。

#### 6.2.2.2 lidar-driver部分

```yaml
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
```

将 ```lidar_type``` 设置为LiDAR类型 。

设置 ```msop_port``` 和 ```difop_port``` 为雷达数据端口号。

#### 6.2.2.3 lidar-ros部分

```yaml
ros:
  ros_frame_id: rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets    
  ros_send_point_cloud_topic: /rslidar_points     
```

将 ```ros_send_point_cloud_topic``` 设置为发送点云的话题。 

### 6.2.3 运行

运行程序。


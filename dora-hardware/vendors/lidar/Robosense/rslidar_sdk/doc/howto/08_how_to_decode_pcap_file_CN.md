# 8 如何解码PCAP文件



## 8.1 简介

本文档展示如何解码PCAP文件, 并发送点云数据到ROS。 

在阅读本文档之前，请确保已阅读雷达用户手册和 [参数简介](../intro/02_parameter_intro_CN.md) 。



## 8.2 步骤

### 8.2.1 获取数据的端口号

请参考雷达用户手册，或者使用第三方工具（WireShark等）抓包，得到雷达的目标MSOP端口和目标DIFOP端口。端口的默认值分别为`6699`和`7788`。

### 8.2.2 设置参数文件

设置参数文件```config.yaml```。

#### 8.2.2.1 common部分

```yaml
common:
  msg_source: 3                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

消息来自PCAP包，所以设置 ```msg_source = 3``` 。

将点云发送到ROS以便查看，所以设置 ```send_point_cloud_ros = true``` 。 

#### 8.2.2.2 lidar-driver部分

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
      pcap_path: /home/robosense/lidar.pcap        
```

将```pcap_path``` 设置为PCAP文件的全路径。

将 ```lidar_type``` 设置为LiDAR类型。

设置 ```msop_port``` 和 ```difop_port``` 为雷达数据的目标端口号，这里分别是6699和7788。

#### 8.2.2.3 lidar-ros部分

```yaml
ros:
  ros_frame_id: rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets    
  ros_send_point_cloud_topic: /rslidar_points     
```

将 ```ros_send_point_cloud_topic``` 设置为发送点云的话题，这里是/rslidar_points。 

### 8.2.3 运行

运行程序。

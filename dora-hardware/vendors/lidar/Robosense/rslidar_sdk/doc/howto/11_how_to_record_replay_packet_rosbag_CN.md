# 11 如何录制与回放 Packet rosbag



## 11.1 简介

本文档展示如何记录与回放MSOP/DIFOP rosbag。 

使用ROS可以录制点云rosbag消息并回放，但点云包非常大，所以rslidar_sdk提供更好的选择，也就是录制Packet rosbag并回放。

在阅读本文档之前, 请先阅读雷达用户手册和 [连接在线雷达并发送点云到ROS](./06_how_to_decode_online_lidar_CN.md) 。



## 11.2 录制

### 11.2.1 将MSOP/DIFOP Packet发送至ROS

这里假设已经连接在线雷达，并能发送点云到ROS。

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: true                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

要录制Packet, 设置 ```send_packet_ros = true```。

### 11.2.2 根据话题录制rosbag

修改```ros_send_packet_topic```, 来改变发送的话题。这个话题包括MSOP Packet和DIFOP Packet。

```yaml
ros:
  ros_frame_id: rslidar
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

ROS录制rosbag的指令如下。

```bash
rosbag record /rslidar_packets -O bag
```



## 11.3 回放

假设录制了一个rosbag，其中包含话题为 `/rslidar_packets` 的MSOP/DIFOP Packet。

### 11.3.1 设置Packet源

配置`config.yaml`的`common`部分。

```yaml
common:
  msg_source: 2                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

MSOP/DIFOP Packet来自ROS rosbag，因此设置 ```msg_source = 2``` 。

将点云发送到ROS，因此设置 ```send_point_cloud_ros = true```。

### 11.3.2 设置雷达参数

配置`config.yaml`的`lidar-driver`部分。

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

### 11.3.3 设置接收Packet的主题

设置`config.yaml`的`lidar-ros`部分。

```yaml
ros:
  ros_frame_id: rslidar
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points  
```

将 ```ros_recv_packet_topic``` 设置为rosbag中MSOP/DIFOP Packet的话题。



### 11.3.4 运行

运行程序，回放rosbag。



 

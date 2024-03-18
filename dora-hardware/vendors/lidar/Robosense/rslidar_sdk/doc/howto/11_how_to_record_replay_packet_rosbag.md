# 11 How to record and replay Packet rosbag



## 11.1 Introduction

This document illustrates how to record and replay MSOP/DIFOP Packet rosbag. 

It is possible to record the point cloud message into a rosbag and replay it, but the point cloud rosbag is very large. rslidar_sdk provides a better way -  record packet rosbag and replay it. 

Please be sure you have read the LiDAR user-guide and [Connect to online LiDAR and send point cloud through ROS](./06_how_to_decode_online_lidar.md).



## 11.2 Record

### 11.2.1 Send packet to ROS

Here suppose that you have connected to an on-line LiDAR, and have sent the point cloud to ROS.


```yaml
common:
  msg_source: 1                                       
  send_packet_ros: true                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

To record packets, set ```send_packet_ros = true```. 

### 11.2.2 Record the topic of packet

To change the topic of packet, change ```ros_send_packet_topic```. This topic sends out both MSOP and DIFOP packets. 

```yaml
ros:
  ros_frame_id: rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

Record rosbag as below.

```sh
rosbag record /rslidar_packets -O bag
```



## 11.3 Replay

Suppose you have recorded a rosbag, which contains MSOP/DIFOP packets with the topic ```/rslidar_packets```. 

### 11.3.1 Set Packet Source

In `config.yaml`, set the `common` part.

```yaml
common:
  msg_source: 2                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

Packet is from the ROS, so set ```msg_source = 2```. 

To send point cloud to ROS, set ```send_point_cloud_ros = true```.

### 11.3.2 Set parameters of Lidar

In `config.yaml`, set the `lidar-driver` part.

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

Set the ```lidar_type```  to your LiDAR type.

### 11.3.3 Set Topic of packet.

In `config.yaml`, set the `lidar-ros` part.

```yaml
ros:
  ros_frame_id: rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points  
```

To receive MSOP/DIFOP packest, set ```ros_recv_packet_topic```  to the topic in the rosbag.

### 11.3.4 Run

Run the demo, and replay rosbag.

 

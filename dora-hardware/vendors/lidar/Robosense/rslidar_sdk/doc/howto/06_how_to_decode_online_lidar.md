# 6 How to decode on-line LiDAR

## 6.1 Introduction

This document illustrates how to connect to an on-line LiDAR, and send point cloud to ROS. 

Please make sure you have read the LiDAR user-guide and [Intro to parameters](../intro/02_parameter_intro.md) before reading this document.



## 6.2 Steps

### 6.2.1 Get the LiDAR port number

Please follow the instructions in LiDAR user-guide, to connect the LiDAR, and set up your computer's ip address. 

Please check the LiDAR user-guide, or use the 3rd-party tool(such as WireShark), to get your LiDAR's MSOP port number and DIFOP port number. The default values are ```msop-6699, difop-7788```. 

### 6.2.2 Set up the configuration file

#### 6.2.2.1 common part

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

The message come from the LiDAR, so set ```msg_source = 1```. 

Send point cloud to ROS, so set ```send_point_cloud_ros = true```.

#### 6.2.2.2 lidar-driver part

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

Set the ```msop_port``` and ```difop_port```  to your LiDAR's port number. 

#### 6.2.2.3 lidar-ros part

```yaml
ros:
  ros_frame_id: rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

Set the ```ros_send_point_cloud_topic```  to the topic you want to send to. 

### 6.2.3 Run

Run the program. 


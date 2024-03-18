# 8 How to decode PCAP file



## 8.1 Introduction

This document illustrates how to decode PCAP file, and send point cloud to ROS. 

Please make sure you have read the LiDAR user-guide and [Intro to parameters](../intro/02_parameter_intro.md) before reading this document.



## 8.2 Steps

### 8.2.1 Get the LiDAR port number

Please check the LiDAR user-guide, or use the 3rd-party tool(such as WireShark), to get your LiDAR's MSOP port number and DIFOP port number. The default values are ```msop-6699, difop-7788```. 

### 8.2.2 Set up the configuration file

Set up the configuration file `config.yaml`.

#### 8.2.2.1 common part

```yaml
common:
  msg_source: 3                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

The messages come from the PCAP bag, so set ```msg_source = 3```. 

Send point cloud to ROS, so set ```send_point_cloud_ros = true```. 

#### 8.2.2.2 lidar-driver part

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

Set the ```pcap_path``` to the absolute path of the PCAP file.

Set the ```lidar_type```  to your LiDAR type.

Set the ```msop_port``` and ```difop_port```  to the port numbers of your LiDAR. 

#### 8.2.2.3 lidar-ros part

```yaml
ros:
  ros_frame_id: rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

Set the ```ros_send_point_cloud_topic```  to the topic you want to send to.

### 8.2.3 Run

Run the program. 






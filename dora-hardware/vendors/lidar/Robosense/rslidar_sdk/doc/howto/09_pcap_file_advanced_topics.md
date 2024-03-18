# 9 PCAP File - Advanced Topics



## 9.1 Introduction

The RoboSense LiDAR may work 
+ in unicast/multicast/broadcast mode, 
+ with VLAN layer 
+ with user layers. 
+ Also rslidar_sdk supports multi-LiDARs.

This document illustrates how to configure rslidar_sdk in each case.

Before reading this document, please be sure that you have read:
+ LiDAR user-guide 
+ [Intro to parameters](../intro/02_parameter_intro.md) 
+ [Online LiDAR - Advanced Topics](./07_online_lidar_advanced_topics.md)



## 9.2 General Case

Generally, below code is for decoding a PCAP file in these cases.
+ Broadcast/multicast/unicast mode
+ There are multiple LiDars in a file.

```yaml
common:
  msg_source: 3                                       
  send_point_cloud_ros: true                            

lidar:
  - driver:
      lidar_type: RS32           
      pcap_path: /home/robosense/lidar.pcap
      msop_port: 6699
      difop_port: 7788
    ros:
      ros_frame_id: rslidar           
      ros_send_point_cloud_topic: /rslidar_points     
```

The only exception is "Multiple Lidars with same ports but different IPs", which is not supported now.



## 9.3 VLAN

In some user cases, The LiDar may work on VLAN.  Its packets have a VLAN layer.
![](./img/07_06_vlan_layer.png)

rs_driver decodes PCAP file and gets all parts of MSOP packets, including the VLAN layer. 

To strip the VLAN layer, just set `use_vlan: true`.

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      pcap_path: /home/robosense/lidar.pcap
      msop_port: 6699             
      difop_port: 7788
      use_vlan: true            
```



## 9.4 User Layer, Tail Layer 

In some user cases, User may add extra layers before or/and after the MSOP/DIFOP packet.
+ USER_LAYER is before the packet and TAIL_LAYER is after it.
![](./img/07_08_user_layer.png)

These extra layers are parts of UDP data. The driver can strip them. 

To strip them, just give their lengths in bytes. 

In the following example, USER_LAYER is 8 bytes, and TAIL_LAYER is 4 bytes.

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      pcap_path: /home/robosense/lidar.pcap
      msop_port: 6699             
      difop_port: 7788
      user_layer_bytes: 8
      tail_layer_bytes: 4      
```


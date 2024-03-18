# 3 Introduction to hidden parameters

In order to make the configuration file as simple as possible, we hide some parameters and use default values for them. 

This document explains the meanings of these hidden parameters. 



## 3.1 common

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                                
  send_point_cloud_ros: false                           
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```



## 3.2 lidar

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
      pcap_repeat: true									    
      pcap_rate: 1  	
      config_from_file: false										
      angle_path: /home/robosense/angle.csv   
      dense_points: false
      ts_first_point: false
      split_frame_mode: 1	      
      split_angle: 0   
      num_blks_split: 1 	                    
      wait_for_difop: true         
      group_address: 0.0.0.0
      host_address: 0.0.0.0
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      use_vlan: false
```

- ```pcap_repeat``` -- The default value is ```true```. Set it to ```false``` to prevent play pcap repeatedly.
- ```pcap_rate``` -- The default value is ```1```. The frequency of point cloud is about 10hz. The larger the value is, the faster the pcap bag is played.
- ```config_from_file``` -- Whether to read Lidar configuration from file. Only used for debug purpose, and can be ignored.
- ```angle_path``` -- The path of the angle.csv. Only used for debug purpose and can be ignored.
- ```ts_first_point``` --  Stamp the point cloud with the first point or the last one. Stamp with the first point if ```true```, else stamp with the last point if ```false```. The default value is ```false```. 
- ```split_frame_mode``` -- The way to split the LiDAR frames. Default value is ```1```.
  - 1 -- Split frame depending on the split_angle
  - 2 -- Split frame depending on a fixed number of blocks
  - 3 -- Split frame depending on num_blks_split

- ```split_angle``` --  The angle(in degree) to split frames. Only be used when ```split_frame_mode = 1```. The default value is ```0```.
- ```num_blks_split``` -- The number of blocks in one frame. Only be used when ```split_frame_mode = 3```.
- ```wait_for_difop``` -- If ```false```, the driver will not wait for difop packet(including lidar configuration data, especially angle data to calculate x, y, z), and send out the point cloud immediately. The default value is ```true```.
- ```group_address``` -- If use multi-cast function, this parameter needs to be set correctly. For more details, please refer to  [Online LiDAR - Advanced Topics](../howto/07_online_lidar_advanced_topics.md) 
- ```host_address``` -- Needed in two conditions. If the host receives packets from multiple Lidars via different IP addresses, use this parameter to specify destination IPs of the Lidars; If group_address is set, it should be set, so it will be joined into the multicast group.
- ```x, y, z, roll, pitch, yaw ``` -- The parameters to do coordinate transformation. If the coordinate transformation function is enabled in driver core,  the output point cloud will be transformed based on these parameters. For more details, please refer to [Coordinate Transformation](../howto/10_how_to_use_coordinate_transformation.md) 
- ```use_vlan``` -- Whether to use VLAN. The default value is ```false```. This parameter is only needed for pcap file. If it contains packets with VLAN layer, ```use_vlan``` should set to true. In the case of online Lidar, the VLAN layer is stripped by the protocol layer, so use_vlan can be ignored. 


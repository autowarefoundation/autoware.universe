# 3 隐藏参数介绍

为了使配置文件config.yaml尽可能简洁，我们隐藏了部分不常用的参数，在代码中使用默认值。

本文档将详细介绍这些隐藏参数。用户可根据需要，将它们加入参数文件，重新设置。



## 3.1 common

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                                
  send_point_cloud_ros: false                           
```



## 3.2 lidar

```yaml
lidar:
  - driver:
      lidar_type: RS128            
      msop_port: 6699              
      difop_port: 7788             
      pcap_path: /home/robosense/lidar.pcap                 
      pcap_repeat: true									    
      pcap_rate: 1  											
      start_angle: 0               
      end_angle: 360             
      min_distance: 0.2            
      max_distance: 200           
      use_lidar_clock: false       
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

- ```pcap_repeat``` -- 默认值为true， 用户可将其设置为false来禁用pcap循环播放功能。
- ```pcap_rate``` -- 默认值为1，点云频率约为10hz。 用户可调节此参数来控制pcap播放速度，设置的值越大，pcap播放速度越快。
- ```config_from_file``` -- 默认值为false, 是否从外参文件读入雷达配置信息，仅用于调试，可忽略。
- ```angle_path``` -- angle.csv外参文件的路径，仅用于调试，可忽略。
- ```ts_first_point``` -- 默认值为false。点云的时间戳是否第一个点的时间。```true```为第一个点的时间，```false```为最后一个点的时间。
- ```split_frame_mode``` -- 分帧模式设置，默认值为```1```。
  - 1 -- 角度分帧
  - 2 -- 固定block数分帧
  - 3 -- 自定义block数分帧
- ```split_angle``` --  用于分帧的角度(单位为度)， 在```split_frame_mode = 1``` 时才生效，默认值为```0```。
- ```num_blks_split``` -- 用于分帧的包数，在 ```split_frame_mode = 3```时才生效，默认值为1。
- ```wait_for_difop``` -- 若设置为false， 驱动将不会等待DIFOP包（包含配置数据，尤其是角度信息），而是立即解析MSOP包并发出点云。 默认值为```true```，也就是必须要有DIFOP包才会进行点云解析。
- ```group_address``` -- 如果雷达为组播模式，此参数需要被设置为组播的地址。具体使用方式可以参考[在线雷达 - 高级主题](../howto/07_online_lidar_advanced_topics_CN.md) 。
- ```host_address``` -- 有两种情况需要这个选项。如果主机上通过多个IP地址接收多个雷达的数据，则可以将此参数指定为雷达的目标IP；如果设置了group_address，那也需要设置host_address，以便将这个IP地址的网卡加入组播组。
- ```x, y, z, roll, pitch, yaw ``` -- 坐标变换参数，若启用了内核的坐标变换功能，将会使用此参数输出经过变换后的点云。x, y, z, 单位为```米```, roll, pitch, yaw, 单位为```弧度```。具体使用方式可以参考 [坐标变换功能](../howto/10_how_to_use_coordinate_transformation_CN.md) 。
- ```use_vlan``` -- 默认为false，指定是否使用vlan。如果pcap文件中的packet带vlan层，则需要设置这个选项为true。其他情况下不需要。在线雷达的情况下，协议层到达驱动时，已经剥离vlan层，所以不需要设置这个选项。

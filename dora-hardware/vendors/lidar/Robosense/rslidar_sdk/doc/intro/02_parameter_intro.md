# 2 Introduction to Parameters 

rslidar_sdk reads parameters from the configuration file ```config.yaml```, which is stored in ```rslidar_sdk/config```.  

`config.yaml` contains two parts, the `common` part and the `lidar` part. 

rslidar_sdk supports multi-LiDARs case. The `common` part is shared by all LiDARs, while in the `lidar` part, each child node is for an individual Lidar.

**config.yaml is indentation sensitive! Please make sure the indentation is not changed after adjusting the parameters!**



## 2.1 Common

The `common` part specifies the source of LiDAR packets, and where to publish point clouds and packets.

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: false                           
```

- msg_source

  - 0 -- Unused. Never set this parameter to 0.

  - 1 -- LiDAR packets come from on-line LiDARs. For more details, please refer to [Connect to online LiDAR and send point cloud through ROS](../howto/06_how_to_decode_online_lidar.md)

  - 2 -- LiDAR packets come from ROS/ROS2. It is used to decode from an off-line rosbag. For more details, please refer to [Record rosbag & Replay it](../howto/11_how_to_record_replay_packet_rosbag.md)

  - 3 -- LiDAR packets come from a PCAP bag. For more details, please refer to  [Decode PCAP file and send point cloud through ROS](../howto/08_how_to_decode_pcap_file.md)

- send_packet_ros

   - true -- LiDAR packets will be sent to ROS/ROS2. 

*The ROS Packet message is of a customized message type, so you can't print its content via the ROS `echo` command. This option is used to record off-line Packet rosbags. For more details, please refer to the case of msg_source=2.*

- send_point_cloud_ros

   - true -- The LiDAR point cloud will be sent to ROS/ROS2. 
   
   *The ROS point cloud type is the ROS official defined type -- sensor_msgs/PointCloud2, so it can be visualized on the ROS  `rviz` tool directly. It is not suggested to record the point cloud to rosbag, because its size may be very large. Please record Packets instead. Refer to the case of msg_source=2.*



## 2.2 lidar

The `lidar` part needs to be adjusted for every LiDAR seperately. 

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
      dense_points: false        
      pcap_path: /home/robosense/lidar.pcap                 
    ros:
      ros_frame_id: rslidar           
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
```

- lidar_type

  Supported LiDAR types are listed in the README file.

- msop_port, difop_port

  The MSOP port and DIFOP port to receive LiDAR packets. *If no data is received, please check these parameters first.*

- start_angle, end_angle

  The start angle and end angle of the point cloud, which should be in the range of 0~360°. *`start_angle` can be larger than `end_angle`*.

- min_distance, max_distance

  The minimum distance and maximum distance of the point cloud. 

- use_lidar_clock

  - true -- Use the Lidar clock as the message timestamp
  - false -- Use the host machine clock as the message timestamp

- dense_points

  Whether to discard NAN points. The default value is ```false```.
  - Discard if ```true``` 
  - reserve if ```false```. 

- pcap_path

   The full path of the PCAP file. Valid if msg_source = 3.

- ros_send_by_rows
  
  Meaningful only for Mechanical Lidars, and valid if dense_points = false。
  - true -- send point cloud row by row
  - false -- send point cloud clolumn by column



## 2.3 Examples

### 2.3.1 Single Lidar Case

Connect to 1 LiDAR of RS128, and send point cloud to ROS.

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: true                           
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
    ros:
      ros_frame_id: rslidar           
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
```

### 2.3.2 Multi Lidar Case

Connect to 1 LiDAR of RS128, and 2 LiDARs of RSBP, and send point cloud to ROS.

*Pay attention to the indentation of the `lidar` part*

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: true                           
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
    ros:
      ros_frame_id: rslidar           
      ros_recv_packet_topic: /middle/rslidar_packets    
      ros_send_packet_topic: /middle/rslidar_packets    
      ros_send_point_cloud_topic: /middle/rslidar_points      
  - driver:
      lidar_type: RSBP           
      msop_port: 1990             
      difop_port: 1991            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_frame_id: rslidar           
      ros_recv_packet_topic: /left/rslidar_packets    
      ros_send_packet_topic: /left/rslidar_packets    
      ros_send_point_cloud_topic: /left/rslidar_points      
  - driver:
      lidar_type: RSBP           
      msop_port: 2010             
      difop_port: 2011            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_frame_id: rslidar           
      ros_recv_packet_topic: /right/rslidar_packets    
      ros_send_packet_topic: /right/rslidar_packets    
      ros_send_point_cloud_topic: /right/rslidar_points      
```


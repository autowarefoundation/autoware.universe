# 9 PCAP文件 - 高级主题



## 9.1 简介

RoboSense雷达可以工作在如下场景。
+ 单播/组播/广播模式
+ 运行在VLAN协议上
+ 向Packet中加入用户自己的层
+ 接入多个雷达

本文说明在每种场景下，如何配置rslidar_sdk。

在阅读本文之前，请先阅读：
+ 雷达用户使用手册
+ [参数介绍](../intro/02_parameter_intro_CN.md) 
+ [在线雷达-高级主题](./07_online_lidar_advanced_topics_CN.md)



## 9.2 一般场景

在下列场景下，使用如下配置解码PCAP文件。
+ 广播/组播/单播模式
+ PCAP文件中有多个雷达

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

一个例外是：PCAP文件中有多个雷达数据，但这些雷达目的端口相同，使用不同的目的IP地址来区分。这种情况不支持。



## 9.3 VLAN

有些场景下，雷达工作在VLAN环境下。这时MSOP/DIFOP包带VLAN层，如下图。
![](./img/07_06_vlan_layer.png)

rs_driver使用libpcap库解析PCAP文件，可以得到完整的、包括VLAN层的MSOP/DIFOP包。

要剥除VLAN层，只需要设置`use_vlan: true`。

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

某些场景下，用户可能在MSOP/DIFOP数据前后加入自己的层。
+ USER_LAYER 在MSOP/DIFOP数据之前，TAIL_LAYER在MSOP/DIFOP数据之后。
![](./img/07_08_user_layer.png)

这些层是UDP数据的一部分，所以rs_driver可以自己剥除他们。只需要告诉它每个层的字节数就可以。

如下的例子中，指定USER_LAYER为8字节，TAIL_LAYER为4字节。

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


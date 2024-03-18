# 7 在线雷达 - 高级主题



## 7.1 简介

RoboSense雷达可以工作在如下的场景。

+ 单播/组播/广播模式。
+ 运行在VLAN协议上。
+ 向Packet加入用户自己的层。
+ 接入多个雷达。

本文描述在这些场景下如何配置rslidar_sdk。

在阅读本文档之前， 请确保已经阅读过：
+ 雷达用户手册 
+ [参数介绍](../intro/02_parameter_intro_CN.md) 
+ [连接在线雷达](./06_how_to_decode_online_lidar_CN.md)



## 7.2 单播、组播、广播

### 7.2.1 广播

雷达发送 MSOP/DIFOP Packet到电脑主机。为简单起见，如下的图没有显示DIFOP端口。
+ 雷达发送Packet到 `255.255.255.255` : `6699`, rslidar_sdk绑定到主机的端口 `6699`.
![](./img/07_01_broadcast.png)

如下是配置`config.yaml`的方式。

```yaml
common:
  msg_source: 1                                       
  send_point_cloud_ros: true                            

lidar:
  - driver:
      lidar_type: RS32           
      msop_port: 6699             
      difop_port: 7788            
    ros:
      ros_frame_id: rslidar           
      ros_send_point_cloud_topic: /rslidar_points     
```

这里列出了`common`部分和`lidar-ros`部分的设置。这两部分设置将在本文中后面的例子沿用，不再列出。

### 7.2.2 单播

为了减少网络负载，建议雷达使用单播模式。
+ 雷达发送Packet到 `192.168.1.102` : `6699`, rslidar_sdk绑定端口 `6699`。
![](./img/07_02_unicast.png)

如下是配置`config.yaml`的方式。这实际上与广播的方式一样。

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      msop_port: 6699             
      difop_port: 7788            
```

### 7.2.3 组播

雷达也可以工作在组播模式。
+ 雷达发送Packet到 `224.1.1.1`:`6699` 
+ rslidar_sdk绑定到端口 `6699`。同时它将IP地址为`192.168.1.102`的本地网络接口加入组播组`224.1.1.1`。
![](./img/07_03_multicast.png)

如下是配置`config.yaml`的方式。

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      msop_port: 6699             
      difop_port: 7788
      group_address: 224.1.1.1
      host_address: 192.168.1.102
```



## 7.3 多个雷达的情况

### 7.3.1 不同的目标端口

如果有两个或多个雷达，首选的配置是让它们有不同的目标端口。
+ 第一个雷达发送Packet到 `192.168.1.102`:`6699`, 给rslidar_sdk配置的第一个driver节点绑定到`6699`。
+ 第二个雷达发送Packet到 `192.168.1.102`:`5599`, 给rslidar_sdk配置的第二个driver节点绑定到`5599`。
![](./img/07_04_multi_lidars_port.png)

如下是配置`config.yaml`的方式。

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      msop_port: 6699             
      difop_port: 7788
  - driver:
      lidar_type: RS32           
      msop_port: 5599
      difop_port: 6688
```

### 7.3.2 不同的目标IP

也可以让多个雷达使用不同的目标IP。
+ 主机有两个网卡， IP地址分别为`192.168.1.102` 和 `192.168.1.103`。
+ 第一个雷达发送Packet到 `192.168.1.102`:`6699`, 给rslidar_sdk配置的第一个driver节点绑定到`192.168.1.102:6699`。
+ 第二个雷达发送Packet到 `192.168.1.103`:`6699`, 给rslidar_sdk配置的第二个driver节点绑定到`192.168.1.103:6699`。
![](./img/07_05_multi_lidars_ip.png)

如下是配置`config.yaml`的方式。

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      msop_port: 6699             
      difop_port: 7788
      host_address: 192.168.1.102
  - driver:
      lidar_type: RS32           
      msop_port: 6699
      difop_port: 7788
      host_address: 192.168.1.103
```



## 7.4 VLAN

在某些场景下，雷达工作在VLAN层之上。MSOP/DIFOP Packet有VLAN层，如下图。
![](./img/07_06_vlan_layer.png)

rslidar_sdk不能解析VLAN层。

需要一个虚拟网卡来剥除掉这一层。举例如下。

+ 雷达工作在VLAN id为`80`的VLAN层上。它发送Packet到`192.168.1.102` : `6699`，Packet有VLAN层。
+ 假设主机上有一个支持VLAN的物理网卡`eno1`. 它接收带VLAN层的Packet。
![](./img/07_07_vlan.png)

要剥离VLAN层，需要基于`eno1`，创建一个虚拟网卡`eno1.80`, 并且将它的IP设置为`192.168.1.102`。

```shell
sudo apt-get install vlan -y
sudo modprobe 8021q

sudo vconfig add eno1 80
sudo ifconfig eno1.80 192.168.1.102 up
```

现在，rslidar_sdk可以将`eno1.80`当做一个一般的网卡来处理，从这里接收不带VLAN层的Packet.

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      msop_port: 6699             
      difop_port: 7788            
```



## 7.5 User Layer, Tail Layer 

在某些场景下，用户可能在MSOP/DIFOP数据前后加入自己的层。
+ 在前面的是USER_LAYER，在后面的是TAIL_LAYER。
![](./img/07_08_user_layer.png)

这两个层是UDP数据的一部分，所以rslidar_sdk可以自己剥除它们。只需要指出这两个层的长度就可以了。

在下面的例子中，USER_LAYER是8字节，TAIL_LAYER是4字节。

```yaml
lidar:
  - driver:
      lidar_type: RS32           
      msop_port: 6699             
      difop_port: 7788
      user_layer_bytes: 8
      tail_layer_bytes: 4      
```


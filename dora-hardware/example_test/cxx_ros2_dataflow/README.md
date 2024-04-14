# `cxx_node_rust_api` 样例

这个例子描述了C++与ROS 之间通过  publish/subscribe 和 service的方式通讯.  

## 编译节点

保证在～/dora 目录包含dora github工程；dora V0.3.2 安在 ～/dora_project 下

下载样例

```
mkdir build 
cd build
cmake ..
make
```

## 运行 pub/sub 样例

ROS2客户端发布turtlesim消息到dora节点，dora节点发送控制数据到cmd_vel话题，控制小乌龟运动

```
source /opt/ros/galactic/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run turtlesim turtlesim_node
```

启动dora节点
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/galactic/setup.bash
dora start up dataflow.yml --name test_ros
```
通过log文件查看dora节点输出

```shell
dora logs test_ros cxx_node_rust_api
```

## 运行 service 样例
- 新建终端运行ROS2测试节点
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run demo_nodes_cpp add_two_ints_server
```

启动dora节点
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/galactic/setup.bash
dora start up dataflow.yml --name test_ros
```


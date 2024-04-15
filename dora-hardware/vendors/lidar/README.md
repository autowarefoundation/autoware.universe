# 激光雷达驱动与Rviz可视化

## 文件说明

lidar.pcap:激光雷达数据包

rs_driver:雷达驱动包

rslidar_driver_pcap.cc: C++读取pcap点云数据包

rslidar_driver.cc: 雷达传感器C++驱动代码

rslidar_driver_ros.cc: C++发送到ros2显示点云的代码

## Dora版本

参考：https://dora.carsmos.ai/docs/guides/Installation/installing

```bash
export DORA_VERSION=v0.3.2 # Check for the latest release
export ARCHITECTURE=$(uname -m)
wget https://github.com/dora-rs/dora/releases/download/${DORA_VERSION}/dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
unzip dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
pip install dora-rs==${DORA_VERSION}
PATH=$PATH:$(pwd)
dora --help
```

## 激光雷达驱动安装

参考：https://blog.csdn.net/crp997576280/article/details/135376558
rs_driver git仓库：https://blog.csdn.net/crp997576280/article/details/135376558

```bash
git clone https://github.com/RoboSense-LiDAR/rs_driver.git
# 安装编译所需依赖
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
# 编译rs_driver
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
# 安装rs_driver
sudo make install
```
## 编译激光雷达dora驱动

在dora_to_ros2/lidar目录下执行编译指令

```bash
mkdir build && cd build
cmake ..
cmake --build .
```
## Demo 运行

雷达传感器读取数据的指令:

```bash
dora start dataflow.yml --name test
```

PCAP数据包读取的指令:

```bash
dora start dataflow_pcap.yml --name test
```

PCAP数据包读取发送到ROS2的指令:

```bash
dora start dataflow_pcap_ros --name test
```

## 结果

PCAP文件读取测试:

```bash
dora start dataflow_pcap.yml --name test
```

可以在ros2看见话题： *ros2_bridge/lidar_data*,启动Rviz2，通过话题添加PointCloud2可以显示激光雷达数据。

![1](./figure/1.png)

![lidar_pcar_ros2](/home/crp/autoware.universe/dora-hardware/dora_to_ros2/lidar/figure/lidar_pcar_ros2.gif)
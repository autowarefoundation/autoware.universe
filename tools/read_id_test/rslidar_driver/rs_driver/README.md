# 1 **rs_driver**  

[中文介绍](README_CN.md) 



## 1.1 Introduction

**rs_driver** is the driver kernel for the RoboSense LiDARs.

Please download the official release from [github](https://github.com/RoboSense-LiDAR/rs_driver/releases), or get the latest version with the git client tool.

````shell
git clone https://github.com/RoboSense-LiDAR/rs_driver.git
````



## 1.2 Supported LiDARs

Below are the supported LiDARS.

- RS-LiDAR-16
- RS-LiDAR-32
- RS-Bpearl
- RS-Bpearl-V4
- RS-Helios
- RS-Helios-16P
- RS-Ruby-128
- RS-Ruby-80
- RS-Ruby-48
- RS-Ruby-Plus-128
- RS-Ruby-Plus-80
- RS-Ruby-Plus-48
- RS-LiDAR-M1
- RS-LiDAR-M2
- RS-LiDAR-E1



## 1.3 Supported Platforms

**rs_driver** is supported on the following platforms and compilers. Note the compiler should support `C++14`.

- Ubuntu (16.04, 18.04)
  - gcc (4.8+)

- Windows
  - MSVC ( tested with Win10 / VS2019)



## 1.4 Dependency Libraries

**rs_driver** depends on the following third-party libraries. 

- libpcap (optional, needed to parse PCAP file)
- Eigen3 (optional, needed to use the internal transformation function)
- PCL (optional, needed to build the visualization tool)
- Boost (optional, needed to build the visualization tool)



## 1.5 Compile On Ubuntu

### 1.5.1 Dependency Libraries

```sh
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```

### 1.5.2 Compilation

```bash
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
```

### 1.5.3 Installation

```bash
sudo make install
```

### 1.5.4 Use `rs_driver` as a third party library

In your ```CMakeLists.txt```, find the **rs_driver** package and link to it .

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```

### 1.5.5 Use `rs_driver` as a submodule

Add **rs_driver** into your project as a submodule. 

In your ```CMakeLists.txt```, find the **rs_driver** package and link to it .

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```



## 1.6 Compile On Windows

### 1.6.1 Dependency Libraries

#### 1.6.1.1 libpcap

Install [libpcap runtime library](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe).

Unzip [libpcap's developer's pack](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip) to your favorite location, and add the path to the folder ```WpdPack_4_1_2/WpdPack``` to the environment variable ```PATH``` . 

####  1.6.1.2 PCL

To compile with VS2019, please use the official installation package [PCL All-in-one installer](https://github.com/PointCloudLibrary/pcl/releases).

Select the "Add PCL to the system PATH for xxx" option during the installation.
![](./img/01_01_install_pcl.png)

### 1.6.2 Installation

Installation is not supported on Windows.



## 1.7 Quick Start

**rs_driver** offers two demo programs in ```rs_driver/demo```.

- demo_online.cpp

​	`demo_online` connects to online lidar, and output point cloud.

- demo_pcap.cpp

​	`demo_pcap` parses pcap file, and output point cloud. It is based on libpcap.



To build `demo_online` and `demo_pcap`, enable the CMake option `COMPILE_DEMOS`.

```bash
cmake -DCOMPILE_DEMOS=ON ..
```

For more info about `demo_online`, Please refer to [Decode online LiDAR](doc/howto/07_how_to_decode_online_lidar.md)

For more info about `demo_pcap`, Please refer to [Decode pcap file](doc/howto/09_how_to_decode_pcap_file.md)



## 1.8 Visualization of Point Cloud

**rs_driver** offers a visualization tool `rs_driver_viwer` in ```rs_driver/tool``` , which is based on PCL.

To build it, enable the CMake option CMOPILE_TOOLS.

```bash
cmake -DCOMPILE_TOOLS=ON ..
```

For more info about `rs_driver_viewer`, please refer to [Visualization tool guide](doc/howto/13_how_to_use_rs_driver_viewer.md) 



## 1.9 API files

For more info about the `rs_driver` API, Please refer to:
- **Point Cloud message definition**: ```rs_driver/src/rs_driver/msg/point_cloud_msg.hpp```, ```rs_driver/src/rs_driver/msg/pcl_point_cloud_msg.hpp```
- **API definition**: ```rs_driver/src/rs_driver/api/lidar_driver.hpp```
- **Parameters definition**: ```rs_driver/src/rs_driver/driver/driver_param.hpp```, 
- **Error code definition**: ```rs_driver/src/rs_driver/common/error_code.hpp```



## 1.10 More Topics

For more topics, Please refer to:

+ What is available in the **rs_driver** project? Include examples, tools, documents. 

​	Please see [Directories and Files](doc/intro/02_directories_and_files.md) 

+ There are two versions of **rs_driver**: `v1.3.x` and `v1.5.x`. Which is suggested? Is it suggested to upgrade to `v1.5.x` ? How to upgrade?

​	Please see [How to port from v1.3.x to v1.5.x](doc/howto/07_how_to_port_your_app_from_rs_driver_1.3.x_to_1.5.x.md) 

+ How to compile **rs_driver** on Windows ?

​	Please see [How to compile `rs_driver` on Windows](./doc/howto/16_how_to_compile_on_windows.md)

+ What is the interface of **rs_driver** like ? How to integrate it into my system?

​	Please see [Thread Model and Interface](./doc/intro/03_thread_model.md)

+ Is there any examples about how to integrate **rs_driver** ? And if mulitple LiDARs?

​	Please see [How to connect to online LiDAR](doc/howto/08_how_to_decode_online_lidar.md)，[How to decode PCAP file](doc/howto/10_how_to_decode_pcap_file.md)。

+ How to configure **rs_driver**'s network options in unicast/multicast/broadcast mode? And if mulitple Lidars? With VLAN layer? May I add extra layers before/after MSOP/DIFOP packet? All configurations are OK, and Wireshark/tcpdump can capture the packets, but **rs_driver** still fails to get them. Why ?

​	Please see [Online LiDAR - Advanced Topics](doc/howto/09_online_lidar_advanced_topics.md) 

+ To decode a PCAP file with only one LiDAR data, how to configure `rs_driver`? And if multiple LiDARs? With VLAN layer? With extra layers before/after MSOP/DIFOP packet?

​	Please see [PCAP File - Advanced Topics](doc/howto/11_pcap_file_advanced_topics.md) 

+ Can I determine how to configure `rs_driver` by a PCAP file? For both online LiDAR and PCAP file?

​	Please see [How to configure rs_driver by PCAP file](doc/howto/12_how_to_configure_by_pcap_file.md)

+ What format of PCAP file is supported by `rs_driver`? How to capture such a file?

​	Please see [How to capture a PCAP file for rs_driver](doc/howto/13_how_to_capture_pcap_file.md)

+ How to compile demo apps/tools? How to avoid packet loss? How much is CPU usage? Point cloud is UTC time. How to convert it to loca time? How to disable PCAP file decoding on embedded Linux? How to enable point transformation?

​	Please see [rs_driver CMake Macros](./doc/intro/05_cmake_macros_intro.md)

+ How to specify Lidar type? How to specify data source (online LiDAR, PCAP file, user's data) ? How to timestamp point cloud with LiDAR clock or Host clock? How to discard NAN points in point cloud? Where does point cloud get its timestamp? from its first point, or last point? How to configure point transform? How to change the splitting angle of mechanical LiDAR?

​	Please see [rs_driver configuration parameters](./doc/intro/04_parameter_intro.md)

+ **rs_driver** reports an error `ERRCODE_MSOPTIMEOUT`/`ERRCODE_WRONGMSOPID`/`ERRCODE_PKTBUFOVERFLOW` ...... What does it mean?

​	Please see [rs_driver Error Code](./doc/intro/06_error_code_intro.md)

+ How to transform point cloud to another position?

​	Please see [How to transform point cloud](doc/howto/15_how_to_transform_pointcloud.md) 

+ How much is data flow of RoboSense LiDARs ? When may it loss packets? How to avoid this?

​	Please see [How to avoid packet Loss](doc/howto/17_how_to_avoid_packet_loss.md)

+ How much CPU and memory does **rs_driver** need？

​	Please see [CPU Usage and Memory Usage of `rs_driver`](doc/howto/20_about_usage_of_cpu_and_memory.md)

+ What is point layout like? What's the coordinate system of **rs_driver**? What is `ring` of point? Where is point timestamp from?

​	Please see [Point Layout in point cloud](doc/howto/18_about_point_layout.md)

+ How does RoboSense split frames? It uses the UDP protocol. What if packet loss or out of order?

​	Please see [Splitting frames](doc/howto/19_about_splitting_frame.md)

+ Want to know more implementation details about `rs_driver`?

​	Please see [Analysis of rs_driver's source code](doc/src_intro/rs_driver_intro_CN.md)


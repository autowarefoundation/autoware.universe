# 1 **rs_driver预览**

[English Version](README.md) 



## 1.1 概述

**rs_driver**是RoboSense雷达的驱动库。

可以从[github](https://github.com/RoboSense-LiDAR/rs_driver/releases)下载正式版本。也可以使用git工具得到最新版本。

```
git clone https://github.com/RoboSense-LiDAR/rs_driver.git
```



## 1.2 支持的雷达型号

**rs_driver**支持的雷达型号如下。

+ RS-LiDAR-16
+ RS-LiDAR-32
+ RS-Bpearl
+ RS-Bpearl-V4
+ RS-Helios
+ RS-Helios-16P
+ RS-Ruby-128
+ RS-Ruby-80
+ RS-Ruby-48
+ RS-Ruby-Plus-128
+ RS-Ruby-Plus-80
+ RS-Ruby-Plus-48
+ RS-LiDAR-M1
+ RS-LiDAR-M2
+ RS-LiDAR-E1



## 1.3 支持的操作系统

**rs_driver**支持的操作系统及编译器如下。注意编译器需支持`C++14`标准。

- Ubuntu (16.04, 18.04, 20.04)
  - gcc (4.8+)

- Windows
  - MSVC  (Win10 / VS2019 已测试)



## 1.4 依赖的第三方库

**rs_driver**依赖的第三方库如下。

- `libpcap` (可选。如不需要解析PCAP文件，可忽略)
- `eigen3` (可选。如不需要内置坐标变换，可忽略)
- `PCL` (可选。如不需要可视化工具，可忽略)
- `Boost` (可选。如不需要可视化工具，可忽略)



## 1.5 Ubuntu下的编译及安装

### 1.5.1 安装第三方库

```bash
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```
### 1.5.2 编译

```bash
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
```

### 1.5.3 安装

```bash
sudo make install
```

### 1.5.4 作为第三方库使用

配置您的```CMakeLists.txt```文件，使用find_package()指令找到**rs_driver**库，并链接。

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${rs_driver_LIBRARIES})
```

### 1.5.5 作为子模块使用

将**rs_driver**作为子模块添加到您的工程，相应配置您的```CMakeLists.txt```文件。

使用find_package()指令找到**rs_driver**库，并链接。

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```



## 1.6 Windows下的编译及安装

### 1.6.1 安装第三方库

#### 1.6.1.1 libpcap

安装[libpcap运行库](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe)。

解压[libpcap开发者包](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip)到任意位置，并将```WpdPack_4_1_2/WpdPack``` 的路径添加到环境变量```PATH```。

#### 1.6.1.2 PCL

如果使用MSVC编译器，可使用PCL官方提供的[PCL安装包](https://github.com/PointCloudLibrary/pcl/releases)安装。

安装过程中选择 `Add PCL to the system PATH for xxx`。
![](./img/01_01_install_pcl.png)

### 1.6.2 安装

Windows下，**rs_driver** 暂不支持安装。

### 1.6.3 VS工程文件

工程目录`rs_driver/win`下，有编译实例程序和工具的MSVS工程文件。

关于Windows下编译和使用**rs_driver**的更多说明，可以参考[如何在Windows下编译rs_driver](doc/howto/16_how_to_compile_on_windows_CN.md)



## 1.7 示例程序

**rs_driver**在目录```rs_driver/demo``` 下，提供了两个示例程序。

- `demo_online.cpp`

​	`demo_online`解析在线雷达的数据，输出点云。


- `demo_pcap.cpp`

​	`demo_pcap`解析PCAP文件，输出点云。编译`demo_pcap`需要安装`libpcap`库。



要编译这两个程序，需使能`COMPILE_DEMOS`选项。

```bash
cmake -DCOMPILE_DEMOS=ON ..
```

关于`demo_online`的更多说明，可以参考[如何连接在线雷达](doc/howto/08_how_to_decode_online_lidar_CN.md)

关于`demo_pcap`的更多说明，可以参考[如何解码PCAP文件](doc/howto/10_how_to_decode_pcap_file_CN.md)



## 1.8 点云可视化工具

**rs_driver**在目录```rs_driver/tool``` 下，提供了一个点云可视化工具`rs_driver_viewer`。

要编译这个工具，需使能`COMPILE_TOOS`选项。编译它需要安装PCL库和Boost库。

```bash
cmake -DCOMPILE_TOOLS=ON ..
```

关于`rs_driver_viewer`的使用方法，请参考[如何使用可视化工具rs_driver_viewer](doc/howto/14_how_to_use_rs_driver_viewer_CN.md) 



## 1.9 接口文件

**rs_driver**的主要接口文件如下。

- 点云消息定义: ```rs_driver/src/rs_driver/msg/point_cloud_msg.hpp```, ```rs_driver/src/rs_driver/msg/pcl_point_cloud_msg.hpp```
- 接口定义: ```rs_driver/src/rs_driver/api/lidar_driver.hpp```
- 参数定义: ```rs_driver/src/rs_driver/driver/driver_param.hpp```
- 错误码定义: ```rs_driver/src/rs_driver/common/error_code.hpp```



## 1.10 更多主题

关于**rs_driver**的其他主题，请参考如下链接。

+ **rs_driver**工程中包括哪些例子、工具、和文档？

​	请参考[rs_driver的目录结构](doc/intro/02_directories_and_files_CN.md) 

+ **rs_driver**有`v1.3.x`和`v1.5.x`两个版本？该选哪一个？现在正在使用`v1.3.x`，需要升级到`v1.5.x`吗？如何升级？

​	请参考[如何从v1.3.x升级到v1.5.x](doc/howto/07_how_to_port_your_app_from_rs_driver_1.3.x_to_1.5.x_CN.md) 

+ Windows上如何编译**rs_driver**？

​	请参考[如何在Windows上编译rs_driver](./doc/howto/16_how_to_compile_on_windows_CN.md)

+ **rs_driver**的接口如何设计的？如何将**rs_driver**集成到我的系统中，有什么注意事项吗？

​	请参考[rs_driver的线程模型与接口设计](./doc/intro/03_thread_model_CN.md)

+ **rs_driver**如何集成到我自己的系统中？有参考的例子吗？有多雷达的例子吗？

​	请参考[如何连接在线雷达](doc/howto/08_how_to_decode_online_lidar_CN.md)，[如何解析PCAP文件](doc/howto/10_how_to_decode_pcap_file_CN.md)。

+ 在单播、组播、广播的情况下，应该如何配置**rs_driver**的网络选项？在多雷达的情况下如何配置？在VLAN的情况下如何配置？可以在MSOP/DIFOP数据的前后加入自己的层吗？这些网络配置都确认无误，用抓包软件也能抓到MSOP/DIFOP包，为什么还是接收不到点云？

​	请参考[在线雷达 - 高级主题](doc/howto/09_online_lidar_advanced_topics_CN.md) 

+ 一般情况下，PCAP文件中只有一个雷达的数据，如何配置网络选项？ 如果文件中有多个雷达的数据，如何配置？ 在VLAN的情况下如何配置？可以在MSOP/DIFOP数据的前后加入自己的层吗？

​	请参考[PCAP文件 - 高级主题](doc/howto/11_pcap_file_advanced_topics_CN.md) 

+ 手上只有一个PCAP文件，可以根据它确定**rs_driver**的网络配置选项吗？包括连接在线雷达和解析PCAP文件。

​	请参考[根据PCAP文件确定网络配置选项](doc/howto/12_how_to_configure_by_pcap_file_CN.md)

+ **rs_driver**支持什么格式的PCAP文件？如何录制这样的文件？

​	请参考[如何为rs_driver录制PCAP文件](doc/howto/13_how_to_capture_pcap_file_CN.md)

+ 想编译示例程序/小工具，怎么指定编译选项？ **rs_driver**丢包了，如何解决？**rs_driver**的CPU占用率？点云的时间戳是UTC时间，能改成本地时间吗？嵌入式平台不需要解析PCAP文件，可以不编译这个特性吗？如何启用点云坐标转换功能？

​	请参考[rs_driver CMake编译宏](./doc/intro/05_cmake_macros_intro_CN.md)

+ **rs_driver**如何指定雷达类型？如何指定数据源为在线雷达、PCAP文件或用户数据？ 想先试用一下，但是对雷达作PTP时间同步很麻烦，可以先用电脑的系统时间给点云打时间戳吗？想节省一点内存空间，可以丢弃点云中的无效点吗？点云的时间戳来自它的第一个点，还是最后一个点？可以配置吗？点云坐标转换的位移、旋转参数如何设置？机械式雷达的分帧角度可以设置吗？

​	请参考[rs_driver配置选项](./doc/intro/04_parameter_intro_CN.md)

+ **rs_driver**报告了一个错误`ERRCODE_MSOPTIMEOUT`/`ERRCODE_WRONGMSOPID`/`ERRCODE_PKTBUFOVERFLOW` ...... 这个错误码是什么意思？

​	请参考[rs_driver错误码](./doc/intro/06_error_code_intro_CN.md)

+ 如何将点云变换到另一个位置和角度上去？

​	请参考[点云坐标变换](doc/howto/15_how_to_transform_pointcloud_CN.md) 

+ RoboSense雷达的数据量有多大？在什么情况下可能丢包？怎么避免？

​	请参考[丢包问题以及如何解决](doc/howto/17_how_to_avoid_packet_loss_CN.md)

+ **rs_driver**要占用多少CPU和内存？

​	请参考[rs_driver的CPU和内存占用](doc/howto/20_about_usage_of_cpu_and_memory_CN.md)

+ 点在点云中是怎样布局的？点坐标的参考系是什么？点的通道		号指什么？点的时间戳来自哪里？

​	请参考[点云中点的布局](doc/howto/18_about_point_layout_CN.md)

+ RoboSense雷达怎么分帧？RoboSense使用UDP协议，万一丢包或乱序，会影响**rs_driver**的分帧处理吗？

​	请参考[RoboSense雷达如何分帧](doc/howto/19_about_splitting_frame_CN.md)

+ 希望进一步深入了解**rs_driver**的设计细节？

​	请参考[rs_driver源代码解析](doc/src_intro/rs_driver_intro_CN.md)


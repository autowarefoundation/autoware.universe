# 5 **CMake编译宏介绍**



## 5.1 概述

`rs_driver`工程目录下的`CMakeLists.txt`中，定义了一组条件编译宏，可以改变编译`rs_driver`的目标和行为。



## 5.2 指定编译目标的宏

`rs_driver`的编译目标包括：

+ 示例程序，包括`demo_online`、`demo_pcap`等。
+ 小工具，包括点云可视化工具`rs_driver_viewer`、将点云保存为PCD文件的工具`rs_driver_pcdsaver`等。
+ 基于Google Test的自动化测试用例。



### 5.2.1 COMPILE_DEMOS

COMPILE_DEMOS 指定是否编译示例程序。
+ COMPILE_DEMOS=OFF，不编译。这是默认值。
+ COMPILE_DEMOS=ON，编译。

```
option(COMPILE_DEMOS "Build rs_driver demos" OFF)
```

### 5.2.2 COMPILE_TOOLS

COMPILE_TOOLS指定是否编译小工具。
+ COMPILE_TOOLS=OFF。是否编译小工具，分别取决于COMPILE_TOOLS_VIEWER和COMPILE_TOOLS_PCDSAVER。这是默认值。
+ COMPILE_TOOLS=ON。编译`rs_driver_viewer`和`rs_driver_pcdsaver`，不管COMPILE_TOOLS_VIEWER和COMPILE_TOOLS_PCDSAVER如何设置。

```
option(COMPILE_TOOLS "Build rs_driver tools" OFF)
```

### 5.2.3 COMPILE_TOOLS_VIEWER

COMPILE_TOOLS_VIEWER指定在COMPILE_TOOLS=OFF时，是否编译`rs_driver_viewer`。
+ COMPILE_TOOLS_VIEWER=OFF，不编译。这是默认值。
+ COMPILE_TOOLS_VIEWER=ON，编译。

```
option(COMPILE_TOOL_VIEWER "Build point cloud visualization tool" OFF)
```

### 5.2.4 COMPILE_TOOLS_PCDSAVER

COMPILE_TOOL_PCDSAVER指定在COMPILE_TOOLS=OFF时，是否编译`rs_driver_pcdsaver`。
+ COMPILE_TOOLS_PCDSAVER=OFF，不编译。这是默认值。
+ COMPILE_TOOLS_PCDSAVER=ON，编译。

```
option(COMPILE_TOOL_PCDSAVER "Build point cloud pcd saver tool" OFF)
```

### 5.2.5 COMPILE_TESTS

COMPILE_TESTS 指定是否编译测试用例。
+ COMPILE_TESTS=OFF，不编译。这是默认值。
+ COMPILE_TESTS=ON，编译。

```
option(COMPILE_TESTS "Build rs_driver unit tests" OFF)
```



## 5.3 改变编译行为的宏

### 5.3.1 DISABLE_PCAP_PARSE

DISALBE_PCAP_PARSE 指定是否支持PCAP数据源，也就是从PCAP文件解析MSOP/DIFOP Packet。
+ DISABLE_PCAP_PARSE=OFF，支持PCAP数据源。这是默认值。
+ DISABLE_PCAP_PARSE=ON，不支持。在嵌入式Linux上，一般不需要解析PCAP文件，这个宏可以避免移植第三方的`libpcap`库。

```
option(DISABLE_PCAP_PARSE         "Disable PCAP file parse" OFF) 
```

### 5.3.2 ENABLE_TRANSFORM

ENABLE_TRANSFORM 指定是否支持坐标转换功能。
+ ENABLE_TRANSFORM=OFF，不支持。这是默认值。
+ ENABLE_TRANSFORM=ON，支持。**坐标转换功能显著消耗CPU资源，请不要在正式发布的产品中使用它。**

```
option(ENABLE_TRANSFORM           "Enable transform functions" OFF)
```

### 5.3.3 ENABLE_DOUBLE_RCVBUF

ENABLE_DOUBLE_RCVBUF 指定是否增大接收MSOP/DIFOP的socket的接收缓存。

在某些平台上（如嵌入式Linux、Windows），默认的接收缓存比较小，会导致丢包。这时，使能这个宏可以避免丢包。
+ ENABLE_DOUBLE_RCVBUF=OFF，不增大，保持系统的默认设置。这是默认值。
+ ENABLE_DOUBLE_RCVBUF=ON，增大。目前的实现是改成默认值的`4`倍，`4`倍是针对部分平台测试的结果。如果需要，可以在代码中直接改变这个值。

```
option(ENABLE_DOUBLE_RCVBUF       "Enable double size of RCVBUF" OFF)
```

### 5.3.4 ENABLE_WAIT_IF_QUEUE_EMPTY

ENABLE_WAIT_IF_QUEUE_EMPTY 指定在MSOP/DIFOP Packet队列为空时，`rs_driver`的处理线程等待的方式。
+ ENABLE_WAIT_IF_QUEUE_EMPTY=OFF， 等待条件变量通知。这是默认值。
+ ENABLE_WAIT_IF_QUEUE_EMPTY=ON，调用usleep()。这样处理可以减少CPU资源消耗，但是会增大点云帧的延迟。

是否使能这个宏，需要根据具体的应用场景权衡利弊，再作决定。

```
option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
```

### 5.3.5 ENABLE_EPOLL_RECEIVE

ENABLE_EPOLL_RECEIVE 指定接收MSOP/DIFOP Packet的实现方式。
+ ENABLE_EPOLL_RECEIVE=OFF，使用select()。这是默认值。
+ ENABLE_EPOLL_RECEIVE=ON，使用epoll()。

```
option(ENABLE_EPOLL_RECEIVE       "Receive packets with epoll() instead of select()" OFF)
```

### 5.3.6 ENABLE_STAMP_WITH_LOCAL

ENABLE_STAMP_WITH_LOCAL 指定是否将点云的时间转换为本地时间。
+ ENABLE_STAMP_WITH_LOCAL=OFF，保持UTC时间，不转换。这是默认值。
+ ENABLE_STAMP_WITH_LOCAL=ON，根据时区设置，转换到本地时间。

```
option(ENABLE_STAMP_WITH_LOCAL    "Enable stamp point cloud with local time" OFF)
```

### 5.3.7 ENABLE_PCL_POINTCLOUD

ENABLE_PCL_POINTCLOUD 指定示例程序中的点云格式。
+ ENABLE_PCL_POINTCLOUD=OFF，RoboSense自定义格式。这是默认值。
+ ENABLE_PCL_POINTCLOUD=ON，PCL格式。使能这个选项需要PCL库的支持。

```
option(ENABLE_PCL_POINTCLOUD      "Enable PCL Point Cloud" OFF)
```

### 5.3.8 ENABLE_CRC32_CHECK

ENABLE_CRC32_CHECK 指定对MSOP/DIFOP Packet的数据作CRC32校验。
+ ENABLE_CRC32_CHECK=OFF，不校验。这是默认值。
+ ENABLE_CRC32_CHECK=ON，校验。使能这个选项，需要雷达本身支持这个特性。

```
option(ENABLE_CRC32_CHECK      "Enable CRC32 Check on MSOP Packet" OFF)
```

### 5.3.9 ENABLE_DIFOP_PARSE

ENABLE_DIFOP_PARSE 指定是否解析DIFOP Packet，得到雷达的配置和状态数据。
+ ENABLE_DIFOP_PARSE=OFF，不解析。这是默认值。
+ ENABLE_DIFOP_PARSE=ON，解析。注意这个特性只是解析几个域作为例子。请参考文档 [how to parse difop packet](../howto/21_how_to_parse_difop_CN.md)

```
option(ENABLE_DIFOP_PARSE      "Enable Parsing DIFOP Packet" OFF)
```


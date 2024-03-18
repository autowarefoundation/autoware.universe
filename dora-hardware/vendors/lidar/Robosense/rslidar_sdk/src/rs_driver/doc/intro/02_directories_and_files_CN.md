# 2 **目录结构**

## 2.1 目录列表

`rs_driver`工程的主要目录如下。

+ `src` 库的源代码
+ `demo` 使用rs_driver库的示例程序，包括：
  + `demo_online.cpp` 连接在线雷达的例子
  + `demo_online_muti_lidars.cpp` 连接多个在线雷达的例子
  + `demo_pcap.cpp` 解析PCAP文件的例子
+ `tool` 实用工具程序
  + `rs_driver_viewer.cpp` 基于PCL库的点云可视化工具
  + `rs_driver_pcdsaver.cpp` 将点云保存为PCD格式的工具。在嵌入式环境下可能没有PCL库支持，所以`rs_driver_viewer`不可用。这时可以用`rs_driver_pcdsaver`导出点云。
+ `test` 基于`Google Test`的单元测试 
+ `doc` 帮助文档
  + `howto` 回答了使用`rs_driver`的一些常见问题，如对`demo_online`/`demo_pcap`例子和`rs_driver_viewer`工具的讲解，如何配置网络选项，如何对点云作坐标转换，如何从`v1.3.x`移植到`v1.5.x`，如何分帧，如何处理丢包和乱序，如何给点云打时间戳，点在点云中如何布局等。
  + `intro` 对`rs_driver`的接口说明，包括参数选项、错误码、CMake编译宏等。
  + `src_intro` 解析rs_driver源代码的文档，说明`rs_driver`的设计思路和一些重要的实现细节。
+ `win` Windows下`MSVC`的工程文件，已经设置好编译选项，可以成功编译`demo_online`/`demo_pcap`/`rs_driver_viewer`等。

```
├── src
│   └── rs_driver
├── demo
│   ├── demo_online.cpp
│   ├── demo_online_multi_lidars.cpp
│   └── demo_pcap.cpp
├── tool
│   ├── rs_driver_pcdsaver.cpp
│   └── rs_driver_viewer.cpp
├── test
├── doc
│   ├── howto
│   ├── intro
│   └── src_intro
├── win
├── CMakeLists.txt
├── README_CN.md
├── README.md
├── CHANGELOG.md
├── LICENSE
```


# **17 如何解决丢包问题**



## 17.1 概述

本文分析丢包原因，给出解决丢包问题的方法。



## 17.2 如何确定是否丢包

运行示例程序`demo_online`，观察每帧的点数是否正常。
+ 机械式雷达的每帧点数应该接近理论点数。如果有大幅波动，则可能是丢包了。
+ M1雷达的每帧点数应该就是理论点数，是固定的。如果少了，则可能是丢包了。

要观察多雷达的情况，可以打开多个终端，分别运行多个`demo_online`的实例，看看每个雷达有没有丢包。



## 17.3 丢包原因

以下情况下可能丢包。
+ 在某些平台上，如Windows和嵌入式Linux平台
+ 多雷达的情况下
+ 系统CPU资源紧张时



## 17.4 解决办法

解决丢包的办法是将接收MSOP Packet的Socket的接收缓存增大。

在`rs_driver`工程的`CMakeLists.txt`中，宏`ENABLE_DOUBLE_RCVBUF`可以使能这个特性。

```cmake
option(ENABLE_DOUBLE_RCVBUF       "Enable double size of RCVBUF" OFF)
```

代码如下。建议在实际场景下测试，再根据测试结果，将缓存大小调整为合适的值。

```c++
#ifdef ENABLE_DOUBLE_RCVBUF
  {
    uint32_t opt_val;
    socklen_t opt_len = sizeof(uint32_t);
    getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, &opt_len);
    opt_val *= 2;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char*)&opt_val, opt_len);
  }
#endif
```

在某些平台下，如Ubuntu，系统设置了接收缓存的最大值。如果`rs_driver`设置的值超过了最大值，调用setsockopt()就不会成功。

这时需要手工放宽这个限制，如下面的指令，将这个最大值改为`851,968`个字节。


```shell
sudo bash -c "echo 851968 > /proc/sys/net/core/rmem_max"
```


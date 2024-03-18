# 20 **CPU占用与内存占用**

## 20.1 概述

本文说明`rs_driver`对CPU和内存的使用。

请先阅读[rs_driver的线程模型与接口设计](../intro/03_thread_model_CN.md)，因为本文引用了其中的概念。
![](./img/20_01_components_and_threads.png)



## 20.2 CPU占用

### 20.2.1 CPU占用的来源

`rs_driver`的CPU占用主要来自两个方面：

+ MSOP/DIFOP Packet的处理线程`process_thread`解析MSOP/DIFOP Packet，构建点云
+ `process_thread`等待MSOP/DIFOP Packet时，反复被`唤醒` ->`睡眠` ->`唤醒`。
  + 这个过程CPU占用率较高，原因是Packet是频率高但不连续的，`process_thread`切换状态的次数太多了。



### 20.2.2 降低CPU占用率的办法

CMake编译宏 ENABLE_WAIT_IF_QUEUE_EMPTY 可以让`rs_driver`的CPU占用率降低一点，但是弊端是点云延迟会加大。

它的原理是让`process_thread`每次睡眠的时间长一点，这样每次唤醒时处理的包数就多一些，反复`唤醒` ->`睡眠` ->`唤醒`的次数就少一些。


```cmake
option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
```



打开这个宏后，Packet处理线程会调用usleep()，而不是等待条件变量通知。

在某平台的测试结果表明，这个宏可以将CPU占用率从`25%`降低到`22%`。

```c++
#ifdef ENABLE_WAIT_IF_QUEUE_EMPTY
  
    ...

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    return value;
#else

    ...
    
    std::unique_lock<std::mutex> ul(mtx_);
    cv_.wait_for(ul, std::chrono::microseconds(usec), [this] { return (!queue_.empty()); });

    ...
    
    return value;
#endif
```



这里也有必要说明这样做的弊端。

对于usleep()的等待时间是不确定的。如果等待跨过了点云的分帧点，比如

+ 机械式雷达，MSOP Packet的Block水平角跨过了分帧角度
+ MEMS雷达，比如M1的MSOP Packet序列号跨过了`630`


这样分帧就会拖后，`rs_driver`的使用者“看”到这一帧的时间点就会推迟。

所以请您根据自己的场景，权衡CPU占用和点云延迟之间的利弊，再决定是否使能这个编译宏。



## 20.3 内存占用

### 20.3.1 MSOP/DIFOP Packet队列

`rs_driver`的内存占用主要来自MSOP/DIFOP Packet队列 `free_pkt_queue`和`stuffed_pkt_queue`。

  + 这两个队列中的Packet实例是按需分配的，分配的数量与雷达类型、使用场景相关。
  + 在`rs_driver`的实现中，它不会释放这些实例。如果有偶然的意外发生，导致Packet实例的数量异常增多，那么后面它们所占用的内存也不会降低。

### 20.3.2 点云队列

`rs_driver`使用的点云实例是调用者分配、管理的，所以这里不讨论点云占用的内存。

### 20.3.3 查表方式计算三角函数值

+ 机械式雷达使用查表的方式计算三角函数值。`Trigon`类负责做这件事，它是解码器`Decoder`的成员。

```c++
template <typename T_PointCloud>
class Decoder
{
  ...
  Trigon trigon_;
  ...
};
```

`Trigon`计算(-90, 450)角度范围内的`sin`和`cos`值，粒度是`0.01`度，所以保存它们的两个数组大小为：

```c++
(450 - (-90)) / 0.01 * sizeof(float) * 2 = 432,000 (字节)
```

+ 关于MEMS雷达，
  + M1也需要使用查表方式计算三角函数值，它也使用`Trigon`类，占用内存大小与机械式雷达相同。
  + M2不需要计算三角函数值，也就不需要占用这部分内存。

+ 如果连接多个雷达，`rs_driver`的多个实例各有自己的`Trigon`实例。也就是每个实例都占用`432,000`字节。

### 20.3.4 Socket接收缓存

一个比较隐蔽的内存占用，是接收MSOP/DIFOP Packet的Socket的接收缓存。
+ MSOP Packet和DIFOP Packet一般各有自己接收的Socket
+ 在丢包的情况下，可能需要增加Socket接收缓存的大小


# 20 **CPU Usage and Memory Usage**



## 20.1 Overview

This document illustrates CPU usage and memory usage of `rs_driver`. 

Please first read [thread model and interface](../intro/03_thread_model.md), since this document imports some concept from it.
![](./img/20_01_components_and_threads.png)



## 20.2 CPU Usage

### 20.2.1 What cost CPU resources ?

CPU usage is mainly from two factors.

+ The handing thread `process_thread` parses MSOP/DIFOP Packet, and creates point cloud.
+ While handling MSOP/DIFOP packets, `process_thread` loops in the routine of `waken` ->`sleep` ->`waken`. 
  + This routine cost much CPU resource, because MSOP packets are frequent but not continuous. `process_thread` switches too many times.



### 20.2.2 Solution

CMake macro `ENABLE_WAIT_IF_QUEUE_EMPTY` may lower CPU usage, but increase the latency of point cloud. 

Actually it make `process_thread` sleep a little longer, so the thread handle more packets every time, and switches less.


```cmake
option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
```

With this option enabled, `process_thread` calls usleep() instead of waiting for condition_variable.

Based on a test on a specific platform, this macro lowers CPU usage from `25%` to `22%`.

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



Coins have two sides. The disadvantage is:

usleep() is not definite.  If sleep() acrosses the splitting point, such as

+ Mechanical LiDAR, `Block`'s angle across the splitting angle
+ MEMS LiDAR, For M1, packet number across `630`


splitting is delayed, and then user "see" this frame later. 



## 20.3 Memory Usage

### 20.3.1 MSOP/DIFOP Packet Queue

MSOP/DIFOP Packet queue: `free_pkt_queue` and `stuffed_pkt_queue`.

  + Packets in these queues is allocated on demand. The count depends on LiDAR type and user case.
  + `rs_driver` never releases these packets. 

### 20.3.2 Point Cloud Queue

Point cloud is allocated by user. It is not discussed here.

### 20.3.3 Calculate sin()/cos() by table

+ Mechanical LiDAR calculates sin()/cos() by table. Class `Trigon` wrap this work. It is a member of class `Decoder`.

```c++
template <typename T_PointCloud>
class Decoder
{
  ...
  Trigon trigon_;
  ...
};
```

`Trigon`Calculates sin/cos values in the range (-90, 450), with unit `0.01`degree. The size of two arrays is:

```c++
(450 - (-90)) / 0.01 * sizeof(float) * 2 = 432,000 (bytes)
```

+ MEMS LiDAR
  + M1 use tables and `Trigon` too. It takes same size of memory as mechanical LiDAR does.
  + M2 doesn't need tables. No memory usage for this.

+ To connect to multiple LiDARs,  each  `rs_driver` instance has its own `Trigon` instance. Each instance occupies `432,000` bytes.

### 20.3.4 Socket Receiving Buffer

A hidden memory usage is Socket Receiving Buffer。
+ Each of MSOP/DIFOP Packets has their own sockets.
+ It may be increased in case of packet loss














# 15 **如何对点云作坐标转换**

## 15.1 概述

本文说明如何使用坐标转换功能，将点云变换到另一个坐标系上去。

警告：
+ 坐标转换显著消耗CPU资源。提供这个功能，仅用于测试目的。
+ **不要在发布的产品中使能坐标转换**，除非已经仔细评估，确定系统能容忍这些消耗。



## 15.2 步骤

### 15.2.1 CMake编译宏

要使用坐标转换功能，需要使能CMake编译选项```ENABLE_TRANSFORM=ON```。

```bash
cmake -DENABLE_TRANSFORM=ON ..
```

### 15.2.2 配置参数

配置坐标转换的参数，这些参数的默认值是`0`。
+ x, y, z的单位是`米`
+ yaw, pitch, roll的单位是`弧度`
+ 旋转的顺序是 `yaw -> pitch -> roll`

例子如下。

```c++
RSDriverParam param;                             ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;      /// get packet from online lidar
param.input_param.msop_port = 6699;              ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;             ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;              ///< Set the lidar type. Make sure this type is correct

param.decoder_param.transform_param.x = 1;	     ///< unit: m
param.decoder_param.transform_param.y = 0;	     ///< unit: m
param.decoder_param.transform_param.z = 2.5;	 ///< unit: m
param.decoder_param.transform_param.yaw = 1.57;  ///< unit: radian
param.decoder_param.transform_param.pitch = 0.2; ///< unit: radian
param.decoder_param.transform_param.roll = 0.1;  ///< unit: radian

```


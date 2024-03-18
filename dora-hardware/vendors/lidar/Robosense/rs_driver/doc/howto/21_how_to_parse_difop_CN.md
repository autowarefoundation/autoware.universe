# 21 **如何解析DIFOP中的配置和状态数据**



## 21.1 概述

`rs_driver`的首要目标是解析MSOP/DIFOP包，得到点云。其中MSOP包含点的坐标数据，DIFOP包只包含构造点云的一些参数。

+ 对于机械式雷达，DIFOP包含用于标定的角度数据，和回波模式。
+ 对于MEMS雷达，MSOP包就够了，DIFOP包其实是不需要的。

除构造点云需要的数据之外，DIFOP还包含了一些雷达的配置和状态数据，`rs_driver`似乎是解析这些数据的合适的地方。

遗憾的是，不同类型的雷达，DIFOP格式一般不同，甚至同一类型的雷达，不同项目也可能定制自己的格式，这样`rs_driver`就很难给出一个通用而又完善的实现。所以`rs_driver`只能给出一个折中的方案：它针对DIFOP包中若干通用的字段进行解析。使用者可以根据这些范例来解析其他需要的字段。

本文描述如何一步步修改`rs_driver`，支持解析新的字段。



## 21.2 拓展DIFOP包的定义

DIFOP包的定义在雷达解码器的头文件中。

如果客户定制了自己的DIFOP包格式，则需要拓展原有的定义、甚至将这个定义替换成自己的。

以RS128雷达为例，这个定义在`decoder_RS128.hpp`中。

```c++
// decoder_RS128.hpp

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV2 eth;
  RSFOV fov;
  uint8_t reserved1[2];
  uint16_t phase_lock_angle;
  RSVersionV2 version;
  uint8_t reserved2[229];
  RSSN sn;
  ......

} RS128DifopPkt;
```

对于RSM1雷达，这个定义在`decoder_RSM1.hpp`中。

```c++
// decoder_RSM1.hpp

typedef struct
{
  uint8_t id[8];
  uint8_t reserved1[1];
  uint8_t frame_rate;
  RSM1DifopEther eth;
  RSM1DifopFov fov;
  RSM1DifopVerInfo version;
  RSSN sn;
  ......

} RSM1DifopPkt;
```



## 21.3 扩充DeviceInfo和DeviceStatus

`rs_driver`将解析后的结果保存在两个结构`DeviceInfo`和`DeviceStatus`中，它们定义在`driver_param.hpp`中。

DIFOP包中包含配置和状态数据。

+ `DeviceInfo`这个结构保存配置数据，这部分一般是不变的。
+ `DeviceStatus`是状态数据，这部分是变化的。

```c++
// driver_param.hpp

struct DeviceInfo
{
  uint8_t sn[6];
  uint8_t mac[6];
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
};

struct DeviceStatus
{
  float voltage = 0.0f;
};
```

`Decoder<T_PointCloud>`是所有雷达解码器的基类。它的成员`device_info_`和`device_status_`分别保存了配置和状态数据。

```c++
template <typename T_PointCloud>
class Decoder
{
public:

  ......

  DeviceInfo device_info_;
  DeviceStatus device_status_;

  ......
};
```



## 21.4 解析DIFOP包

雷达解码器负责解码DIFOP包，并将结果保存到`Decoder<T_PointCloud>`的成员`device_info_`和`device_status_`中。

对于RSM1，`DecoderRSM1<T_PointCloud>`的成员函数`decodeDifopPkt()`负责解析DIFOP包。

这个函数是一个虚拟函数，在基类`Decoder<T_PointCloud>`中定义。

```c++
//decoder_RSM1.hpp

template <typename T_PointCloud>
inline void DecoderRSM1<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSM1DifopPkt& pkt = *(RSM1DifopPkt*)packet;
  this->echo_mode_ = this->getEchoMode(pkt.return_mode);
  
#ifdef ENABLE_DIFOP_PARSE
  // device info
  memcpy (this->device_info_.sn, pkt.sn.num, 6),
  memcpy (this->device_info_.mac, pkt.eth.mac_addr, 6),
  memcpy (this->device_info_.top_ver, pkt.version.pl_ver, 5),
  memcpy (this->device_info_.bottom_ver, pkt.version.ps_ver, 5),
  
  // device status
  this->device_status_.voltage = ntohs(pkt.status.voltage_1);
#endif
} 
```

对于机械式雷达，有两个选择。

+ 一个选择是在对应的解码器类的虚拟成员函数`decodeDifopPkt()`中解析。如RS128，就是`DecoderRSM1<T_PointCloud>::decodeDifopPkt()`。

```c++
template <typename T_PointCloud>
inline void DecoderRS128<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{   
  const RS128DifopPkt& pkt = *(const RS128DifopPkt*)(packet);
  this->template decodeDifopCommon<RS128DifopPkt>(pkt);
    
  this->echo_mode_ = getEchoMode (pkt.return_mode);
  this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ?
    (this->blks_per_frame_ << 1) : this->blks_per_frame_;
    
#ifdef ENABLE_DIFOP_PARSE
  // add code to parse RS128-specific fields
#endif  
} 
```

+ 如果解析的字段是所有机械式雷达都有的，那么另一个更好的选择是在DecoderMech<T_PointCloud>::decodeDifopCommon()中解析，雷达解码器类的decodeDifopPkt()会调用这个函数。注意这个函数是以T_Difop为参数的模板函数。

```c++
template <typename T_PointCloud>
template <typename T_Difop>
inline void DecoderMech<T_PointCloud>::decodeDifopCommon(const T_Difop& pkt)
{

 ......
 
 // load angles
  if (!this->param_.config_from_file && !this->angles_ready_)
  {
    int ret = this->chan_angles_.loadFromDifop(pkt.vert_angle_cali, pkt.horiz_angle_cali);
    this->angles_ready_ = (ret == 0);
  }

#ifdef ENABLE_DIFOP_PARSE
  // add code to parse fields which are common to all mechanical LiDARs 

  // device info
  memcpy (this->device_info_.sn, pkt.sn.num, 6),
  memcpy (this->device_info_.mac, pkt.eth.mac_addr, 6),
  memcpy (this->device_info_.top_ver, pkt.version.top_ver, 5), 
  memcpy (this->device_info_.bottom_ver, pkt.version.bottom_ver, 5),
  
  // device status
  this->device_status_.voltage = ntohs(pkt.status.vol_12v);
#endif
}
```



## 21.5 读取配置和状态数据

使用者的代码创建`LidarDriver<T_PointCloud>`的实例，所以可以调用它的成员函数`getDeviceInfo()`和`getDeviceStatus()`得到配置和状态数据。

如前所述，`Decoder<T_PointCloud>`的成员`device_info_`和`device_status_`保存这两类数据。

这里需要注意的是，使用者的代码需要检查这两个成员的返回值，来确定数据是不是有效的。也就是DIFOP是不是解析好了。

```c++
template <typename T_PointCloud>
class LidarDriver
{
public:

  ......
  
  inline bool getDeviceInfo(DeviceInfo& info)
  {
    return driver_ptr_->getDeviceInfo(info);
  }

  inline bool getDeviceStatus(DeviceStatus& status)
  {
    return driver_ptr_->getDeviceStatus(status);
  }

  ......
};
```



## 21.6 使能ENABLE_DIFOP_PARSE

一般的使用者只需要解析点云，所以默认情况下希望免去这些CPU资源的消耗。
CMake编译选项`ENABLE_DIFOP_PARSE`用于指定是否解析DIFOP包。如果希望解析，则需要启用这个选项。这个选项定义在`CMakeLists.txt`下。


```cmake
CMakeLists.txt
option(ENABLE_DIFOP_PARSE         "Enable DIFOP Packet Parse" OFF)
```



## 21.7 对RS16/RS32雷达的特别处理

RS16/RS32是早期的雷达，它们的DIFOP包的定义与后来的雷达差别较大。

为了能适配到`DecoderMech<T_PointCloud>::decodeDifopCommon()`，`rs_driver`给他们定义了一个通用的DIFOP包定义，也就是`AdapterDifopPkt`。

```c++
typedef struct
{
  uint16_t rpm;
  RSEthNetV1 eth;
  RSFOV fov;
  RSVersionV1 version;
  RSSN sn; 
  uint8_t return_mode;
  RSStatusV1 status;
  RSCalibrationAngle vert_angle_cali[32];
  RSCalibrationAngle horiz_angle_cali[32];
} AdapterDifopPkt;
```

RS16/RS32的解码器类会先将需要的字段，从自己的DIFOP包中复制到AdapterDifopPkt的实例，再将这个实例交给`DecoderMech<T_PointCloud>::decodeDifopCommon()`处理。

要让RS16/RS32支持新的字段，

+ 首先需要拓展AdapterDifopPkt的定义。

+ 然后需要将这些新的字段复制到AdapterDifopPkt的实例。以RS16为例，在函数RS16DifopPkt2Adapter()中处理。

```c++
inline void RS16DifopPkt2Adapter (const RS16DifopPkt& src, AdapterDifopPkt& dst)
{
  dst.rpm = src.rpm;
  dst.fov = src.fov;
  dst.return_mode = src.return_mode;
  dst.sn = src.sn;
  dst.eth = src.eth;
  dst.version = src.version;
  dst.status = src.status;
  
  ......
}
```


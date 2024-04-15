# 21 **How to Parse DIFOP packet**



## 21.1 Overview

`rs_driver` is supposed to parse MSOP/DIFOP packets and get point cloud. It gets the coordinate data from the MSOP packets, and gets only only some calibration parameters from the DIFOP packets.

+ For mechanical LiDARs, DIFOP contains data about angles and wave mode.
+ For MEMS LiDARs, DIFOP is not needed, and MSOP is enough. 

Furthermore, the DIFOP packet also contains some configuration data and status data of the LiDAR device. `rs_dirver` seems to be a good candidate to parse them.

Unfortunately, the format of DIFOP packet is different between different types of LiDARs and different projects. It is very hard for `rs_driver` to give a common and complete implementation.

The compromise scheme is: `rs_driver` parses only some common fields from DIFOP, and users folllow these examples to add other user-specific fields.


This document illustrates how to change `rs_driver` to parse new fields.



## 21.2 Extend the definition of DIFOP packet 

This definition is in the header file of the decoder. User should extend it, and even replace it with a new one.

For RS128, The definition is in `decoder_RS128.hpp`.

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

For RSM1, the definition is in `decoder_RSM1.hpp`.

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



## 21.3 Extend DeviceInfo and DeviceStatus

`rs_driver` saves the parsing result in two structures: `DeviceInfo` and `DeviceStatus`. They are in `driver_param.hpp`.
+ `DeviceInfo` is supposed to save configuration data. It is stable in general.
+ `DeviceStatus` is supposed to save status data. It is variable.

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

`Decoder<T_PointCloud>` is the base class of all LiDAR decoders. Its members, `device_info_` and `device_status_`, saves configuration data and status data.

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



## 21.4 Parse DIFOP packet

The Decoder parses DIFOP packets and save the result into the members of `Decoder<T_PointCloud>`: `device_info_` and `device_status_`.

For RSM1，the member of `DecoderRSM1<T_PointCloud>::decodeDifopPkt()`, parses DIFOP packets.

This function is a virtual function. It is defined in the base class `Decoder<T_PointCloud>`.

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

For mechanical LiDARs, follow one of these two ways.

+ One is to parse in the specific Decoder. For RS128, it is `DecoderRSM1<T_PointCloud>::decodeDifopPkt()`.

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

+ If the field is common for all mechanical LiDARS, better parse in `DecoderMech<T_PointCloud>::decodeDifopCommon()`. The specific decoder will call it. 

Note this is a template function, and `T_Difop` is its template parameter.

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



## 21.5 Read configuration data and status data

User creates an instance of `LidarDriver<T_PointCloud>`, and call its member funcitons, `getDeviceInfo()` and `getDeviceStatus()`, to get configuration status data.

As describes above, the members of `Decoder<T_PointCloud>`, `device_info_` and `device_status_` save them.

Note user need to check the result value of these functions to confirm the validation of the data. This is to say, whether DIFOP is parsed successfully already or not.

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



## 21.6 Enable ENABLE_DIFOP_PARSE

In general, User needs only point cloud, and avoids parsing DIFOP packets. 

CMake compilation option `ENABLE_DIFOP_PARSE` specify whether to parse them. If so, enable it. 

This option is in `CMakeLists.txt`.


```cmake
CMakeLists.txt
option(ENABLE_DIFOP_PARSE         "Enable DIFOP Packet Parse" OFF)
```



## 21.7 Special steps for RS16/RS32

RS16/RS32 are the earliest LiDARs. Their packet formats are different with other LiDARS.

To adapt to `DecoderMech<T_PointCloud>::decodeDifopCommon()`，`rs_driver` gives a common definition of DIFOP packet: `AdapterDifopPkt`.

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

Decoders of RS16/RS32 first copy the fields from their own packet to the instance of `AdapterDifopPkt`, and dispatch it to `DecoderMech<T_PointCloud>::decodeDifopCommon()`.

To add new fields for RS16/RS32.

+ First extend the definition of `AdapterDifopPkt`.

+ Copy these new fields to the instance of `AdapterDifopPkt`. For RS16, do this in `RS16DifopPkt2Adapter()`.

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


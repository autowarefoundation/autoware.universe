# 4 Introduction to rs_driver's Parameters 



## 4.1 Parameter File

The parameters are defined in the file `rs_driver/src/rs_driver/driver_param.h`.

Basically, there are 3 structures: 

+ RSDriverParam 
+ RSDecoderParam 
+ RSInputParam



## 4.2 RSDriverParam

RSDriverParam contains RSDecoderParam and RSInputParam.

```c++
typedef struct RSDriverParam
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  RSInputParam input_param;
  RSDecoderParam decoder_param;
} RSDriverParam;
```

+ lidar_type - Lidar Type. Some LiDARs are mechanical, and some are MEMS. Some parameters of RSDecoderParam is only for mechanical LiDARs.

```c++
enum LidarType
{
  RS16 = 1,
  RS32,
  RSBP,
  RS128,
  RS80,
  RSHELIOS,
  RSM1
};
```

+ input_type - What source the Lidar packets is from.
  + ONLINE_LIDAR means from online LiDAR; PCAP_FILE means from PCAP file, which is captured with 3rd party tool; RAW_PACKET is user's own data captured with the `rs_driver` API.

```c++
enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};
```



## 4.3 RSInputParam

RSInputParam specifies the detail paramters of packet source.

The following parameters are for `ONLINE_LIDAR` and `PCAP_FILE`.
+ msop_port - The UDP port on the host, to receive MSOP packets.
+ difop_port - The UDP port on the host, to receive DIFOP Packets.

The following parameters are only for ONLINE_LIDAR.
+ host_address - The host's IP, to receive MSOP/DIFOP Packets
+ group_address - A multicast group to receive MSOP/DIFOP packts. `rs_driver` make `host_address` join it.

The following parameters are only for PCAP_FILE.
+ pcap_path - Full path of the PCAP file.
+ pcap_repeat - Whether to replay PCAP file repeatly
+ pcap_rate - `rs_driver` replay the PCAP file by the theological frame rate. `pcap_rate` gives a rate to it, so as to speed up or slow down.
+ use_vlan - If the PCAP file contains VLAN layer, use `use_vlan`=`true` to skip it.

```c++
typedef struct RSInputParam
{
  uint16_t msop_port = 6699;
  uint16_t difop_port = 7788;
  std::string host_address = "0.0.0.0";
  std::string group_address = "0.0.0.0";

  // The following parameters are only for PCAP_FILE
  std::string pcap_path = "";
  bool pcap_repeat = true;
  float pcap_rate = 1.0;
  bool use_vlan = false;
} RSInputParam;

```



## 4.4 RSDecoderParam

RSDecoderParam specifies how rs_driver decode LiDAR's packets.

```c++
typedef struct RSDecoderParam
{
  bool use_lidar_clock = false;
  bool dense_points = false;
  bool ts_first_point = false;
  bool wait_for_difop = true;
  RSTransformParam transform_param;
  bool config_from_file = false;
  std::string angle_path = "";
  float min_distance = 0.2f;
  float max_distance = 200.0f;

  // The following parameters are only for mechanical Lidars.
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;
  float split_angle = 0.0f;
  uint16_t num_blks_split = 1;
  float start_angle = 0.0f;
  float end_angle = 360.0f;

} RSDecoderParam;
```

The following parameters are for all LiDARs。
+ use_lidar_clock - Where the point cloud's timestamp is from. From the LiDAR, or from `rs_driver` on the host? 
  + If `use_lidar_clock`=`true`，use the LiDAR timestamp, else use the host one.
+ dense_points - Whether the point cloud is dense.
  + If `dense_points`=`false`, then point cloud contains NAN points, else discard them.
+ ts_first_point - Whether to stamp the point cloud with the first point, or the last point.
  + If `ts_first_point`=`false`, then stamp it with the last point, else with the first point。
+ wait_for_difop - Whether wait for DIFOP Packet before parse MSOP packets.
  + DIFOP Packet contains angle calibration parameters. If it is unavailable, the point cloud is flat.
  + If you get no point cloud, try `wait_for_difop`=`false`. It might help to locate the problem.
+ transform_param - paramters of coordinate transformation. It is only valid when the CMake option `ENABLE_TRANSFORM`=`ON`.

```c++
typedef struct RSTransformParam
{
  float x = 0.0f;      ///< unit, m
  float y = 0.0f;      ///< unit, m
  float z = 0.0f;      ///< unit, m
  float roll = 0.0f;   ///< unit, radian
  float pitch = 0.0f;  ///< unit, radian
  float yaw = 0.0f;    ///< unit, radian
} RSTransformParam;
```

+ config_from_file - Where to get LiDAR configuration parameters, from extern files, or from DIFOP packet. Internal use only.
+ angle_path - File of angle calibration parameters. Internal use only.
+ min_distance、max_distance - Set measurement range. Internal use only.

The following parameters are only for mechanical LiDARs.
+ split_frame_mode - How to split frame.
  + `SPLIT_BY_ANGLE` is by a user requested angle. User can specify it. This is default and suggested.
  + `SPLIT_BY_FIXED_BLKS` is by blocks theologically; 
  + `SPLIT_BY_CUSTOM_BLKS` is by user requested blocks. 

```c++
enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_BLKS,
  SPLIT_BY_CUSTOM_BLKS
};
```
+ split_angle - If `split_frame_mode`=`SPLIT_BY_ANGLE`, then `split_angle` is the requested angle to split.
+ num_blks_split - If `split_frame_mode`=`SPLIT_BY_CUSTOM_BLKS`，then `num_blks_split` is blocks.

+ start_angle、end_angle - Generally, mechanical LiDARs's point cloud's azimuths are in the range of [`0`, `360`]. Here you may assign a smaller range of [`start_angle`, `end_angle`).


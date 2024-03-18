# 7 **如何从rs_driver v1.3.x升级到v1.5.x**



## 7.1 目前的分支与版本

目前github上的[rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)工程主要有如下分支。

+ `dev`分支： `v1.3.x`的开发分支，最后版本是`v1.3.2`。这个分支已经停止开发。
+ `dev_opt`分支： 在`v1.3.x`基础上重构的`v1.5.x`的开发分支。
+ `main`分支： 稳定分支。以前跟踪`dev`分支，现在跟踪`dev_opt`分支。
![](./img/07_01_branches.png)

一般情况下，`dev_opt`分支中的修改，在基本稳定后，会合并到`main`分支中，并建立新的版本号。

特别地，对一些影响较大的问题，会推出紧急修复补丁，这个补丁会同时合并到`dev_opt`分支和`main`分支中。



## 7.2 v1.5.x相对于v1.3.x的改进

相对于`v1.3.x`，rs_driver `v1.5.x`的主要改进是明显减少CPU占用率。

如下是在某平台上针对`dev`分支和`dev_opt`分支进行对比测试的结果。

`dev`分支的cpu占用率如下图。
![](./img/07_02_cpu_usage_dev.png)

`dev_opt`分支的cpu占用率如下图。
![](./img/07_03_cpu_usage_dev_opt.png)

`dev_opt`分支的CPU占用率比`dev`分支降低了约`5%`，降低的比例为`16.7%`。



其他的改进还包括：

+ 去除对Boost库的依赖，更容易移植到一些嵌入式Linux平台上

+ 增加可选的编译选项，解决在某些平台上丢包的问题

+ 重构网络部分和解码部分的代码，完整支持不同场景下的配置

+ 对于机械式雷达，将分帧的粒度从Packet细化为Block，避免分帧角度附近点云闪烁的问题

+ 完善帮助文档，尤其是提供了[rs_driver源代码解析](../src_intro/rs_driver_intro_CN.md)文档，帮助用户深入理解驱动的实现，以方便功能裁剪。

对于新客户，推荐使用`v1.5.x`。对于老客户，如果有以上问题的困扰，也推荐升级到`v1.5.x`。



## 7.3 如何升级？

相对于`v1.3.x`， `v1.5.x`调整了接口的定义，也调整了部分配置选项的名字和位置。对这些调整的说明如下。

`rs_driver`的接口包括了两个部分：`RSDriverParam`和`LidarDriver`。

### 7.3.1 RSDriverParam

#### 7.3.1.1 RSDriverParam

RSDriverParam包括一些配置选项，这些选项可以改变`rs_driver`的行为。

在`v1.3.x`中，RSDriverParam定义如下。

```c++
typedef struct RSDriverParam
{
  RSInputParam input_param;                ///< Input parameter
  RSDecoderParam decoder_param;            ///< Decoder parameter
  std::string angle_path = "null";         ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string frame_id = "rslidar";        ///< The frame id of LiDAR message
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  bool wait_for_difop = true;              ///< true: start sending point cloud until receive difop packet
  bool saved_by_rows = false;              ///< true: the output point cloud will be saved by rows (default is saved by columns)
};
```

在`v1.5.x`中，RSDriverParam定义如下。

```c++
typedef struct RSDriverParam
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  RSInputParam input_param;                ///< Input parameter
  RSDecoderParam decoder_param;            ///< Decoder parameter
};
```

变动如下：
+ 加入新成员`input_type`。这个成员指定数据源的类型：
  + `ONLINE_LIDAR` 在线雷达, 
  + `PCAP_FILE` PCAP文件
  + `RAW_PACKET` 用户自己保存的MSOP/DIFOP Packet数据

指定了哪种类型，就从哪类数据源获得数据。用户自己的数据在`v1.3.x`中有一组专门的函数处理，在`v1.5.x`中则统一到数据源类型`RAW_PACKET`。

+ 成员`wait_for_difop` 指定在处理MSOP包之前，是否先等待DIFOP包。对于机械式雷达，如果没有DIFOP包中的垂直角信息，点云就是扁平的。这个选项与解码相关，所以把它移到RSDecoderParam中。

+ 删除成员`saved_by_rows`。雷达每轮扫描得到一组点（每个通道一个点），依次保存在点云的vector成员中，这是`按列保存`。在`v1.3.x`中，成员`saved_by_rows` 指定`按行保存`，也就是每次保存一个通道上的所有点。删除这个成员有两个原因：
  + 在`按列保存`的vector中，按照行的顺序检索点，也是容易的。跳过通道数就可以了。
  + `v1.3.x`实现`saved_by_rows`的方式是将点云重排一遍，这样复制点云的代码太大。
  
  如果您的代码依赖`按行保存`这个特性，建议按`跳过通道数`访问达到同样的效果。
  
  作为一个特例，如果您的代码基于ROS，那么适配ROS的驱动`rslidar_sdk`有个选项`ros_send_by_rows`，也可以做到`按行保存`。`rslidar_sdk`本来就需要将`rs_driver`点云转换到ROS点云，行列变换同时进行，不会带来额外的复制成本。
  
+ 成员`angle_path`指定`垂直角/水平角的修正值`从指定的文件读取，而不是解析DIFOP包得到。这个选项是关于数据源的，所以移到RSInputParam。

+ 成员`frame_id` 来自ROS的概念，`rs_driver`本身不需要它，所以删除。适配ROS的驱动 `rslidar_sdk`会创建和处理它。

#### 7.3.1.2 RSInputParam

在`v1.3.x`中，RSInputParam定义如下。

```c++
typedef struct RSInputParam  ///< The LiDAR input parameter
{
  std::string device_ip = "192.168.1.200";     ///< Ip of LiDAR
  std::string multi_cast_address = "0.0.0.0";  ///< Address of multicast
  std::string host_address = "0.0.0.0";        ///< Address of host
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  bool read_pcap = false;          ///< true: The driver will process the pcap through pcap_path. false: The driver will
                                   ///< Get data from online LiDAR
  double pcap_rate = 1;            ///< Rate to read the pcap file
  bool pcap_repeat = true;         ///< true: The pcap bag will repeat play
  std::string pcap_path = "null";  ///< Absolute path of pcap file
  bool use_vlan = false;           ///< Vlan on-off
  bool use_someip = false;         ///< Someip on-off
  bool use_custom_proto = false;   ///< Customer Protocol on-off
}；
```

在`v1.5.x`中，RSInputParam定义如下。

```c++
typedef struct RSInputParam  ///< The LiDAR input parameter
{
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  std::string host_address = "0.0.0.0";        ///< Address of host
  std::string group_address = "0.0.0.0";       ///< Address of multicast group
  std::string pcap_path = "";                  ///< Absolute path of pcap file
  bool pcap_repeat = true;                     ///< true: The pcap bag will repeat play
  float pcap_rate = 1.0f;                      ///< Rate to read the pcap file
  bool use_vlan = false;                       ///< Vlan on-off
  uint16_t user_layer_bytes = 0;    ///< Bytes of user layer. thers is no user layer if it is 0
  uint16_t tail_layer_bytes = 0;    ///< Bytes of tail layer. thers is no tail layer if it is 0
};
```

变动如下：
+ 删除成员`device_ip`。rs_driver接收MSOP/DIFOP包时，其实不需要这个值。
+ 成员`multi_cast_address`改名为`group_address`。新名字想表达的含义是：在组播模式下，`rs_driver`会将IP地址为`host_address`的网卡加入`group_address`指定的组播组。
+ 删除成员`read_pcap`。 既然`RSDriverParam.input_type` 已经明确了数据源类型，`read_pcap`就不需要了。
+ 删除成员`use_someip` and `use_custom_proto`。它们的功能已经被新的选项 `RSInputParam.user_layer_bytes`所取代。

#### 7.3.1.3 RSDecoderParam

在`v1.3.x`中，RSDecoderParam定义如下。

```c++
typedef struct RSDecoderParam  ///< LiDAR decoder parameter
{
  float max_distance = 200.0f;                                       ///< Max distance of point cloud range
  float min_distance = 0.2f;                                         ///< Minimum distance of point cloud range
  float start_angle = 0.0f;                                          ///< Start angle of point cloud
  float end_angle = 360.0f;                                          ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by cut_angle;
                                                                     ///< 2: Split frames by fixed number of packets;
                                                             ///< 3: Split frames by custom number of packets (num_pkts_split)
  uint32_t num_pkts_split = 1;         ///< Number of packets in one frame, only be used when split_frame_mode=3
  float cut_angle = 0.0f;              ///< Cut angle(degree) used to split frame, only be used when split_frame_mode=1
  bool use_lidar_clock = false;        ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  RSTransformParam transform_param;    ///< Used to transform points
  RSCameraTriggerParam trigger_param;  ///< Used to trigger camera
};
```

在`v1.5.x`中，RSDecoderParam定义如下。

```c++
typedef struct RSDecoderParam  ///< LiDAR decoder parameter
{
  bool config_from_file = false; ///< Internal use only for debugging
  std::string angle_path = "";   ///< Internal use only for debugging
  bool wait_for_difop = true;    ///< true: start sending point cloud until receive difop packet
  float min_distance = 0.0f;     ///< Minimum distance of point cloud range
  float max_distance = 200.0f;   ///< Max distance of point cloud range
  float start_angle = 0.0f;      ///< Start angle of point cloud
  float end_angle = 360.0f;      ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  
                                 ///< 1: Split frames by split_angle;
                                 ///< 2: Split frames by fixed number of blocks;
                                 ///< 3: Split frames by custom number of blocks (num_blks_split)
  float split_angle = 0.0f;      ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
  uint16_t num_blks_split = 1;   ///< Number of packets in one frame, only be used when split_frame_mode=3
  bool use_lidar_clock = false;  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points = false;     ///< true: discard NAN points; false: reserve NAN points
  RSTransformParam transform_param; ///< Used to transform points
};
```

变动如下：
+ 成员`cut_angle`改名为`split_angle`, 以便与成员 `split_frame_mode`的名字保持一致。
+ 增加成员`config_from_file`。只有这个成员为`true`，成员`angle_path`才有效。
+ 成员`num_pkts_split`改名为 `number_blks_split`. 因为在`v1.3.x`中，分帧的最小单位是Packet，而在`v1.5.x`中，分帧单位改成了更小的Block。



### 7.3.2 LidarDriver

创建LidarDriver实例，并调用它的成员函数，可以运行`rs_driver`。

在`v1.3.x`中，LidarDriver定义如下。

```c++
template <typename PointT>
class LidarDriver
{
public:

  inline void regRecvCallback(const std::function<void(const PointCloudMsg<PointT>&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }
  ...
};
```


在`v1.5.x`中，LidarDriver定义如下。

```c++
template <typename T_PointCloud>
class LidarDriver
{
public:

 inline void regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud)
  {
    driver_ptr_->regPointCloudCallback(cb_get_cloud, cb_put_cloud);
  }
  ...
};
```

变动如下：
+ LidarDriver类的模板参数从点 `PointT`改成了点云 `T_PointCloud`。
+ LidarDriver的一个点云回调函数，变成了两个。
  + 在`v1.3.x`中，LidarDriver只需要一个点云回调函数`callback`。rs_driver通过这个函数将构建好的点云返回给调用者。
  + 在`v1.5.x`中，LidarDriver需要两个回调函数。一个函数`cb_put_cloud`与`v1.3.x`的类似，rs_driver将构建好的点云返回给调用者。另一个函数`cb_get_cloud`则是`rs_driver`需要通过这个函数，从调用者获取空闲的点云实例。关于这一点的更详细说明，请参考[连接在线雷达](./08_how_to_decode_online_lidar_CN.md) 和 [rs_driver的线程模型与接口设计](../intro/03_thread_model_CN.md)。


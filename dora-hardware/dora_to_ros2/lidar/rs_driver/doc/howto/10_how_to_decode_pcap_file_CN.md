#  10 **如何解码PCAP文件**



## 10.1 概述

本文说明了如何使用`rs_driver`解码PCAP文件。

关于如何获得PCAP文件，请参考[为rs_driver录制PCAP文件](./13_how_to_capture_pcap_file_CN.md)

这里的点和点云定义，引用了`rs_driver`工程的消息文件。

 ```rs_driver/src/rs_driver/msg/point_cloud_msg.hpp```, ```rs_driver/src/rs_driver/msg/pcl_point_cloud_msg.hpp```



## 10.2 步骤

### 10.2.1 定义点

点是点云的基本组成单元。`rs_driver`支持点的如下成员。
- x -- 坐标X，类型float
- y -- 坐标Y，类型float
- z -- 坐标Y，类型float
- intensity -- 反射率，类型uint8_t
- timestamp -- 时间戳，类型double
- ring -- 通道编号。以RS80为例，通道编号的范围是 0~79 (从下往上)。

如下是几个例子。

- 点的成员包括 **x, y, z, intensity**

  ```c++
  struct PointXYZI
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
  };
  ```
  
- 如果使用PCL库，也可以简单使用PCL的点定义 **pcl::PointXYZI**

  ```c++
  typedef pcl::PointXYZI PointXYZI; 
  ```

- 点的成员包括 **x, y, z, intensity, timestamp, ring**

  ```c++
  struct PointXYZIRT
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
  };
  ```

这些点的定义，使用者可以加入新成员，删除成员，改变成员顺序，但是不可以改变成员的类型。

### 10.2.2 定义点云

如下是点云的定义。

  ```c++
  template <typename T_Point>
  class PointCloudT
  {
  public:
    typedef T_Point PointT;
    typedef std::vector<PointT> VectorT;

    uint32_t height = 0;    ///< Height of point cloud
    uint32_t width = 0;     ///< Width of point cloud
    bool is_dense = false;  ///< If is_dense is true, the point cloud does not contain NAN points
    double timestamp = 0.0; ///< Timestamp of point cloud
    uint32_t seq = 0;       ///< Sequence number of message

    VectorT points;
  };
  
  ```
这个点云的定义，使用者可以加入新成员，改变成员顺序，但是不可以删除成员，或者改变成员的类型。

这个点云定义是一个模板类，还需要指定一个点类型作为模板参数。

  ```c++
  typedef PointXYZI PointT;
  typedef PointCloudT<PointT> PointCloudMsg;
  ```

### 10.2.3 定义LidarDriver对象

LidarDriver类是`rs_driver`的接口类。这里定义一个LidarDriver的实例。

```c++
int main()
{
  LidarDriver<PointCloudMsg> driver;          ///< Declare the driver object
  ...
}
```

### 10.2.4 配置LidarDriver的参数

RSDriverParam定义LidarDriver的参数。这里定义RSDriverParam变量，并配置它。

+ `InputType::PCAP_FILE` 意味着解析PCAP文件得到MSOP/DIFOP包。这里的PCAP文件是`/home/robosense/lidar.pcap`。
+ `LidarType::RS16`是雷达类型
+ 分别设置接收MSOP/DIFO包的端口号。

```c++
int main()
{
  ...
  RSDriverParam param;                             ///< Create a parameter object
  param.input_type = InputType::PCAP_FILE;         /// get packet from the pcap file 
  param.input_param.pcap_path = "/home/robosense/lidar.pcap";  ///< Set the pcap file path
  param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
  param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
  param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct
  ...
}
```

### 10.2.5 定义和注册点云回调函数

+ `rs_driver`需要调用者通过回调函数，提供空闲的点云实例。这里定义这第一个点云回调函数。

```c++
SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;

std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}
```

+ `rs_driver`通过回调函数，将填充好的点云返回给调用者。这里定义这第二个点云回调函数。

```c++
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);

  RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
}
```

注意这两个回调函数都运行在`rs_driver`的MSOP/DIFOP Packet的处理线程中，所以它们不可以做太耗时的任务，否则会导致MSOP/DIFOP Packet不能及时处理。

+ 使用者在自己的线程中，处理点云。

```c++
void processCloud(void)
{
  while (1)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the point cloud msg, even it is time-consuming.
    RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

    free_cloud_queue.push(msg);
  }
}

```

+ 注册两个点云回调函数。

```c++
int main()
{
  ...
  driver.regPointCloudCallback(driverReturnPointCloudToCallerCallback, 
                               driverReturnPointCloudToCallerCallback);
  ...
}
```

### 10.2.6 定义和注册异常回调函数

+ `rs_driver`检测到异常发生时，通过回调函数通知调用者。这里定义异常回调函数。

```c++
void exceptionCallback(const Error &code)
{
  RS_WARNING << "Error code : " << code.toString() << RS_REND;
}
```

再一次提醒，这个回调函数运行在`rs_driver`的线程中，所以不可以做太耗时的任务，否则可能导致MSOP/DIFOP包不能及时接收和处理。

+ 在主函数中，注册异常回调函数。

```c++
int main()
{
  ...
  driver.regExceptionCallback(exceptionCallback);  ///<Register the exception callback function
  ...
}
```

### 10.2.7 初始化LidarDriver对象

按照RSDriverParam指定的配置，初始化LidarDriver对象。

```c++
int main()
{
  ...
  if (!driver.init(param))  ///< Call the init function with the parameter
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }
  ...
}
```

### 10.2.8 启动LidarDriver

启动LidarDriver对象。

```c++
int main()
{
  ...
  driver.start();  ///< Call the start function. The driver thread will start
  ...
}
```

### 10.2.9 祝贺

编译`demo_pcap`并运行。您应该可以看到类似如下点云打印了。

```c++
RoboSense Lidar-Driver Linux online demo start......
msg: 0 point cloud size: 96
msg: 1 point cloud size: 28800
msg: 2 point cloud size: 28800
msg: 3 point cloud size: 28800
msg: 4 point cloud size: 28800
msg: 5 point cloud size: 28800
msg: 6 point cloud size: 28800
msg: 7 point cloud size: 28832
msg: 8 point cloud size: 28800
msg: 9 point cloud size: 28800

```

如果您没有看到，可能是因为网络选项的配置不正确，请参考[PCAP文件-高级主题](11_pcap_file_advanced_topics_CN.md)，获得正确的配置。


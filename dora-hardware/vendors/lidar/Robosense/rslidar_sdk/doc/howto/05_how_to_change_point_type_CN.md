# 5 如何改变点类型的定义



## 5.1 简介

本文档介绍如何改变点类型的定义。

在项目的```CMakeLists.txt```文件中设置`POINT_TYPE`变量。修改后，需要重新编译整个工程。

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```



## 5.2 XYZI

`POINT_TYPE`为`XYZI`时，rslidar_sdk使用RoboSense自定义的点类型```PointXYZI```. 

```c++
struct PointXYZI
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};
```

rslidar_sdk将基于`PointXYZI`的点云，转换为ROS的`PointCloud2`消息，再发布出去。

```c++
 sensor_msgs::PointCloud2 ros_msg;
 addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
 addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
 addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
 addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);

 // 
 // copy points from point cloud of `PointXYZI` to `PointCloud2`
 //
 ...
```

这里`PointCloud2`中的`intensity`是`float`类型，而不是`uint8_t`类型。这是因为大多数基于ROS的程序都希望`float`类型的`intensity`。



## 5.3 XYZIRT

`POINT_TYPE`为`XYZIRT`时，rslidar_sdk使用RoboSense自定义的点类型```PointXYZRT```。

```c++
struct PointXYZIRT
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint16_t ring;
  double timestamp;
};
```

rslidar_sdk将基于`PointXYZIRT`的点云，转换为ROS的PointCloud2消息，再发布出去。

```c++
 sensor_msgs::PointCloud2 ros_msg;
 addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
 addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
 addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
 addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
#ifdef POINT_TYPE_XYZIRT
 sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
 sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif

 // 
 // copy points from point cloud of `PointXYZIRT` to `PointCloud2`
 //
 ...
```


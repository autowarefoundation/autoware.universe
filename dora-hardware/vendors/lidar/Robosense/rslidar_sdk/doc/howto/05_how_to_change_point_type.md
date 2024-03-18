# 5 How to change point type



## 5.1 Introduction

This document illustrates how to change the point type. 

In ```CMakeLists.txt``` of the project, change the variable `POINT_TYPE`. Remember to **rebuild** the project after changing it.

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```



## 5.2 XYZI

If `POINT_TYPE` is `XYZI`, rslidar_sdk uses the RoboSense defined type as below. 

```c++
struct PointXYZI
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};
```

rslidar_sdk transforms point cloud of `PointXYZI` to ROS message of `PointCloud2`，and publish it.

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

Here `intensity` of `PointCloud2` is `float` type, not `uint8_t`. This is because most ROS based applications require `intensity` of `float` type.  



## 5.3 XYZIRT

If `POINT_TYPE` is `XYZIRT`, rslidar_sdk uses the RoboSense defined type as below.

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

rslidar_sdk transforms point cloud of `PointXYZIRT` to ROS message of `PointCloud2`，and publish it.

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


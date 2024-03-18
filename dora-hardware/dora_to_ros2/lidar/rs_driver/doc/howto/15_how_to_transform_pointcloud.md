# 15 **How to transform point cloud**



## 15.1 Introduction

This document illustrate how to transform the point cloud to a different position with the built-in transform function.

Warning: 

+ It costs much CPU resources. This function is only for test purpose.  
+ **Never enable this function in the released products**.



## 15.2 Steps

### 15.2.1 Compile

To enable the transformation function, compile the driver with the option ENABLE_TRANSFORM=ON.

```bash
cmake -DENABLE_TRANSFORM=ON ..
```

### 15.2.2 Config parameters

Configure the transformation parameters. These parameters' default value is ```0```.  

+ The unit of x, y, z, is ```m``` 
+ the unit of roll, pitch, yaw, is ```radian```

+ The rotation order of the transformation is **yaw - pitch - row**. 

Below is an example with x=1, y=0, z=2.5, roll=0.1, pitch=0.2, yaw=1.57. 

```c++
RSDriverParam param;                            ///< Create a parameter object
param.input_type = InputType::ONLINE_LIDAR;     /// get packet from online lidar
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct

param.decoder_param.transform_param.x = 1;	  ///< unit: m
param.decoder_param.transform_param.y = 0;	  ///< unit: m
param.decoder_param.transform_param.z = 2.5;	  ///< unit: m
param.decoder_param.transform_param.roll = 0.1; ///< unit: radian
param.decoder_param.transform_param.pitch = 0.2;///< unit: radian
param.decoder_param.transform_param.yaw = 1.57; ///< unit: radian

```


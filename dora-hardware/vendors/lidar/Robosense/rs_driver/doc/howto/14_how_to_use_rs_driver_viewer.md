# 14 **How to visualize point cloud with `rs_driver_viewer`**



## 14.1 Introduction

The `rs_driver_viewer` is a visualization tool for the point cloud. This document illustrates how to use it.
![](./img/14_01_rs_driver_viewer_point_cloud.png)



## 14.2 Compile and Run

Compile the driver with the option COMPILE_TOOLS=ON. 

```bash
cmake -DCOMPILE_TOOS=ON ..
```

Run the tool.

```bash
./tool/rs_driver_viewer 
```

### 14.2.1 Help Menu

- -h

   print the argument menu 

- -type

   LiDAR type, the default value is *RS16*

- -pcap

   Full path of the PCAP file. If this argument is set, the driver read packets from the pcap file, else from online Lidar. 

- -msop

   MSOP port number of LiDAR, the default value is *6699*

- -difop

   DIFOP port number of LiDAR, the default value is *7788*
   
- -group

   the multicast group address

- -host

   the host address

- -x

   Transformation parameter, default is 0, unit: m

- -y

   Transformation parameter, default is 0, unit: m

- -z

   Transformation parameter, default is 0, unit: m

- -roll

   Transformation parameter, default is 0, unit: radian

- -pitch

   Transformation parameter, default is 0, unit: radian

- -yaw

   Transformation parameter, default is 0, unit: radian

Note:

**The point cloud transformation function is available only if the CMake option ENABLE_TRANSFORM is ON.**



## 14.3 Examples

- Decode from an online RS128 LiDAR. Its MSOP port is ```9966```, and DIFOP port is ```8877```

  ```bash
  rs_driver_viewer -type RS128 -msop 9966 -difop 8877 
  ```

- Decode from a PCAP file with RSHELIOS LiDAR data.

  ```bash
  rs_driver_viewer -type RSHELIOS -pcap /home/robosense/helios.pcap
  ```

- Decode with the coordinate transformation parameters: x=1.5, y=2, z=0, roll=1.57, pitch=0, yaw=0

  ```bash
  rs_driver_viewer -type RS16 -x 1.5 -y 2 -roll 1.57 
  ```

  


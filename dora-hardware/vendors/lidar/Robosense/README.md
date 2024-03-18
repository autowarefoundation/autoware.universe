# Dora platform robosense lidar driver guide
This document uses the DeepL translator to translate.
# 1 How to use
## 1.1 Install the lidar and connect the wires as instructed in the manual
## 1.2 Setting up lidar information
Lines 162-166 of rs_drive_dora.cpp, filled in separately:
``` cpp
  param.input_type = InputType::ONLINE_LIDAR; // Input source, ONLINE_LIDAR means that the input source is a certain radar. You can choose to input from a file, but I didn't test that.
  param.input_param.msop_port = 6699; // msop packet input port, default is 6699, see robosense liadr user manual for details.
  param.input_param.difop_port = 7788; // difop packet input port, default 7788, see robosense lidar user manual for details.
  param.lidar_type = LidarType::RSHELIOS_16P; // Lidar type, see the LidarType enum for details.
```
If you need to change other information, such as lidar rotation speed, I'm sorry to say you'll have to write your own. I'm sorry that the answer is because I don't know how to do that.
## 1.3 Compiling as a dynamic library
I use something like this
``` bash
clang++ -c rs_driver_dora.cpp -o build/rs_driver_dora.o -fdeclspec -fPIC -I <current path>/rs_driver/src
clang++ -shared build/rs_driver_dora.o -o build/librs_driver_dora.so
```
I don't know why the robosense LIDAR SDK just writes include in pointed brackets <>, if the compiler reports that it can't find the file, try tuning the ``` -I``` agrues.
## 1.4 Putting it in a dora dataflow
My test dataflow looks like this:
``` yml
nodes:
  - id: op_1
    operator:
        shared-library: build/rs_driver_dora
        inputs:
          tick: dora/timer/millis/100 # inputs every 100 milliseconds because the radar I have on hand spins every 100 milliseconds and outputs one frame of point cloud, please set it according to your radar.
        outputs:
          - pointCloud
  - id: op_2
    operator:
        shared-library: build/point_cloud_test
        inputs:
          tick: op_1/pointCloud

```
Please write your own yml according to your needs.

# 2 Output formats
The output things of the current version of dora is all byte streams.
Regarding the coordinates of the output points, the results of the test on the RSHELIOS_16P Lidar are as follows: with the logo face pointing forward and the output aerial plug connector pointing down, **the X-axis is oriented forwards, the Y-axis is oriented to the left, and the Z-axis is oriented upwards（meters）, which is different from what is stated in the user's manual!** The intensity is an unsigned integer up to 255.It is recommended to use a tool such as RSView (point cloud visualization tool written by the manufacturer) to determine the coordinate direction before using the radar.
## 2.1 Default point cloud output
The default output is a point cloud encoded as a stream of bytes, the point cloud contains all points returned by a one-circle rotation of the lidar (360° frame). The output format is:
|4 bytes point cloud number (unsigned int) |4 bytes unused | 8 bytes for timestamp (double) | a large number of points, 16 byte for each |
Each point is formatted as:
|4 bytes for x-axis coordinates (float) |4 bytes for y-axis coordinates (float) |4 bytes for z-axis coordinates (float) |1 byte reflectivity (uint_8) |3 bytes unused|
or
``` cpp
struct PointXYZI
{
  float x;
  float y;
  float z;
  uint8_t intensity
};
```
You might ask what all those useless bytes for. The answer is because of byte alignment, which makes memory-io about 20% faster and takes up an extra 23% of memory.
## 2.2 Output as a single point
This driver can **theoretically** output all the points in a point cloud frame (360° frame) at a time, each point has not only coordinates and intensity, but also the ring and timestamp of this point. I call it "theoretically" because I haven't tested this feature.
If you want to use this feature, on line 29 you have to add
```cpp
typedef PointXYZI PointT; // Change PointXYZI to PointXYZIRT.
```
The point output format is
| a large number of points, each 24bytes|
The point format:
``` cpp
struct PointXYZIRT
{
  float x.
  float y.
  float z; 
  uint8_t intensity.
  //Attation! There is one byte unused because of byte alignment here, this byte is left empty and has no effect
  uint16_t ring;
  double timestamp; 
}; 
```

# 3 How to rewrite the driver
If you need to make changes to this driver, it is highly recommended that you to read the robosense LIDAR SDK documentation, which should be in the ```./rs_driver/doc``` directory. In particular, the ```03_thread_model.md``` documentation describes the SDK architecture idea.
The SDK has roughly the following functionality built in:
+ Reading data from lidar/from files.
+ Compatible with all robosense lidars.
+ Support radar output ip and port customization
+ Vlan support
+ Multi-lidar support
+ Support for installation position correction (coordinate transformation)
+ Point cloud visualization software RSView
By calling the SDK appropriately, it should be easy to use these functions.
# 4 Others tips
This driver is based on ```./rs_driver/demo/demo_online.cpp ```, it is also recommended to check this file before modification.
This driver has only one modification to the robosense LIDAR SDK: it is located in ```./rs_driver/src/rs_driver/api/lidar_driver.hpp ```line 87, change the return value of the ```regExceptionCallback ``` function to string, because the original function prints an error message through the console.
# Related links
[Official Documentation Download](https://www.robosense.cn/resources-81)

[SDKgithub](https://github.com/RoboSense-LiDAR/rs_driver/releases)

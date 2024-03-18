# dora平台robosense激光雷达驱动
# 1 怎么用
## 1.1 按说明书指示安装激光雷达并连线

## 1.2 可能遇到的问题

1.可能是dora版本问题，头文../../../apis/c/operator/operator_types.h中没有对Input和Output结构体的具体定义，可以参考以下修改：

```
typedef struct Input{
Vec_uint8_t id;
Vec_uint8_t data;
} Input_t;

typedef struct Output{
Vec_uint8_t id;
Vec_uint8_t data;
} Output_t;
```

2.找不到头文件"../../../dora/apis/c/operator/operator_api.h"

可以在代码rs_drive_dora.cpp 里修改为自己文件的绝对路径，以下是参考路径。

```
#include "/home/crp/dora_project/dora-rs/dora/apis/c/operator/operator_api.h"
```

或者编译的时候在clang++后面添加以下内容，以下是参考路径，具体以自己文件夹路径为准。

```
-I /home/crp/dora_project/dora-rs/dora-hardware/vendors/lidar/
```

## 1.3设置雷达信息

rs_drive_dora.cpp 的第162-166行，分别填写：
``` cpp
  param.input_type = InputType::ONLINE_LIDAR;  // 输入源，ONLINE_LIDAR是指输入源是某个雷达，其他的还可以选从文件输入，不过我没测试
  param.input_param.msop_port = 6699;          // msop包输入端口，默认6699，详见robosense激光雷达用户手册
  param.input_param.difop_port = 7788;         // difop包输入端口，默认7788，详见robosense激光雷达用户手册
  param.lidar_type = LidarType::RSHELIOS_16P;  // 雷达型号，具体可以选什么，自己去看LidarType 这个枚举类型
```
如果需要修改其他信息，比如雷达转速之类，很抱歉你得自己写这个功能了，能力所限。
## 1.4 编译为动态库
我使用的是这样
``` bash
cd ~/dora_project/dora-rs/dora-hardware/vendors/lidar/Robosense #请替换为自己的文件夹路径
clang++ -c rs_driver_dora.cpp -o build/rs_driver_dora.o -fdeclspec -fPIC -I /home/crp/dora_project/dora-rs/dora-hardware/vendors/lidar/Robosense/rs_driver/src/  #生成静态库
clang++ -shared build/rs_driver_dora.o -o build/librs_driver_dora.so  #生成动态库
clang++ rs_driver_dora.cpp -lm -lrt -ldl -pthread -lpcap -std=c++14 -L ../../target/release --output build/rs_driver_dora -I /home/crp/dora_project/dora-rs/dora-hardware/vendors/lidar/Robosense/rs_driver/src/ #生成可执行文件
```
如果编译器汇报找不到文件，请尝试在``` -I``` 后面加上相应路径，

## 1.5 放在dora dataflow里用
我的测试dataflow是这样的：
``` yml
nodes:
  - id: op_1
    operator:
        shared-library: build/rs_driver_dora
        inputs:
          tick: dora/timer/millis/100 #每100毫秒一个输入是因为，我手头的雷达每100毫秒转一圈、输出一帧点云，请根据自己的雷达情况设置
        outputs:
          - pointCloud
  - id: op_2
    operator:
        shared-library: build/point_cloud_test
        inputs:
          tick: op_1/pointCloud
```
请用户根据自身需要编写自己的yml
# 2 输出格式
dora目前版本的输出全部是字节流。
关于输出的点的坐标，在RSHELIOS_16P激光雷达上测试的结果为：以商标面指向前、输出航插接口指向下时，**X轴正方向向前，Y轴正方向向左，Z轴正方向向上，单位为米，与用户手册上的说明不同！** 反射率为最大255的无符号整数。建议使用雷达前，先使用RSView（厂商编写的点云可视化工具）等工具确定坐标方向。

## 2.1 默认点云输出
默认情况下输出的是一个编码为字节流的点云，点云包含雷达旋转一周（360°的帧）所回报的所有点。输出格式为：
|4 byte 点云编号（unsigned int）|4 byte 无作用| 8 byte 时间戳（double）|一大堆点，每个16 byte|
每个点的格式为：
|4 byte x轴坐标（float）|4 byte y轴坐标（float）|4 byte z轴坐标（float）|1 byte 反射率（uint_8）|3 byte 无作用|
或者说是

``` cpp
struct PointXYZI
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};
```
你可能会问里面那些无作用的byte是干嘛的，答案是为了字节对齐，对齐之后读写能快大概20%，额外占用了23%的内存。
# 2.2 以单个点的形式输出
本驱动理论上可以每次输出一个点云帧（360°的帧）里的所有点，每个点不止有坐标和反射率，还有它自己出自哪一线、这个点的时间戳。之所以称其为“理论上”是因为这个功能我没有测试。
如果要使用这个功能的话，在第29行

```cpp
typedef PointXYZI PointT;  // 把 PointXYZI 改成 PointXYZIRT
```
点输出格式为
|一大堆点，每个24byte|
点的格式：
```cpp
struct PointXYZIRT
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  //注意这里有一个byte的字节对齐，这个字节空着无作用
  uint16_t ring;
  double timestamp;
};
```
# 3 怎么改
如果您需要对本驱动进行修改，强烈建议您阅读robosense激光雷达SDK文档，应当在```./rs_driver/doc```目录下。尤其是```03_thread_model.md``` 文档描述了SDK架构思路。
SDK大致内置了以下功能：
+ 从激光雷达/从文件读取数据
+ 兼容所有robosense激光雷达
+ 支持雷达输出ip、端口自定义
+ 支持Vlan
+ 支持多雷达
+ 支持安装位置矫正（坐标变换）
+ 点云可视化软件RSView
通过恰当地调用SDK，应当可以简单地实现这些功能
# 4 其他
本驱动是基于```./rs_driver/demo/demo_online.cpp ```修改而来，修改前亦建议查看此文件。
本驱动对robosense激光雷达SDK仅进行了一处修改：位于```./rs_driver/src/rs_driver/api/lidar_driver.hpp ```中第87行，将```regExceptionCallback ```函数的返回值修改为string。其原因为：原本的函数通过控制台打印错误信息。
# 相关链接
[官方文档下载](https://www.robosense.cn/resources-81)

[SDKgithub](https://github.com/RoboSense-LiDAR/rs_driver/releases)

# gnss_poser

功能：借助GeographicLib库将GNSS 模块经纬度坐标转化为东天坐标。

## 1 GeographicLib 依赖


GeographicLib库源文件放置于gnss_poser/third_party文件目录下，只需要在 gnss_poser 目录下执行

```
./build_third_party.sh
```

即可编译生成 libGeographicLib.so 文件



~~**旧版本库安装方式（不需要使用）：**~~

```bash
sudo apt-get update
sudo apt-get install libgeographic-dev geographiclib-tools
git clone https://github.com/geographiclib/geographiclib.git
cd geographiclib
sudo geographiclib-get-geoids all
sudo cp -r /usr/local/share/GeographicLib/geoids/ /usr/share/GeographicLib/
```

## 2 在dora中启动 gnss_poser

**step1:** 编译

```
mkdir build
cd build
cmake ..
make 
```

**step2 ：** 启动 gnss_poser 节点

```
sudo chmod 777 /dev/ttyUSB0
dora start dataflow.yml --name test_gnss
```

新建rviz2终端即可看到GNSS模块输出在局部坐标系下的轨迹了



# autoware 原始文档

## Purpose

The `gnss_poser` is a node that subscribes gnss sensing messages and calculates vehicle pose with covariance.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name             | Type                          | Description                                                                                                     |
| ---------------- | ----------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `~/input/fix`    | `sensor_msgs::msg::NavSatFix` | gnss status message                                                                                             |
| `~/input/navpvt` | `ublox_msgs::msg::NavPVT`     | position, velocity and time solution. [click here for more details](https://github.com/KumarRobotics/ublox.git) |

### Output

| Name                     | Type                                            | Description                                                    |
| ------------------------ | ----------------------------------------------- | -------------------------------------------------------------- |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`               | vehicle pose calculated from gnss sensing data                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | vehicle pose with covariance calculated from gnss sensing data |
| `~/output/gnss_fixed`    | `tier4_debug_msgs::msg::BoolStamped`            | gnss fix status                                                |

## Parameters

### Core Parameters

| Name                 | Type   | Default Value    | Description                                                                                                                                |
| -------------------- | ------ | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| `base_frame`         | string | "base_link"      | frame id                                                                                                                                   |
| `gnss_frame`         | string | "gnss"           | frame id                                                                                                                                   |
| `gnss_base_frame`    | string | "gnss_base_link" | frame id                                                                                                                                   |
| `map_frame`          | string | "map"            | frame id                                                                                                                                   |
| `coordinate_system`  | int    | "4"              | coordinate system enumeration; 0: UTM, 1: MGRS, 2: Plane, 3: WGS84 Local Coordinate System, 4: UTM Local Coordinate System                 |
| `use_ublox_receiver` | bool   | false            | flag to use ublox receiver                                                                                                                 |
| `plane_zone`         | int    | 9                | identification number of the plane rectangular coordinate systems. [click here for more details](https://www.gsi.go.jp/LAW/heimencho.html) |


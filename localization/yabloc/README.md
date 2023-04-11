# YabLoc

**YabLoc** is camera-baed localization with vector map.

This has been developed as a new location stack for [Autoware](https://github.com/autowarefoundation/autoware).

![thumbnail](docs/yabloc_thumbnail.jpg)

## Submodule

* [external/autoware_auto_msgs](https://github.com/tier4/autoware_auto_msgs)
* [external/autoware_msgs](https://github.com/autowarefoundation/autoware_msgs.git)
* [external/septentrio_gnss_driver](https://github.com/tier4/septentrio_gnss_driver.git)
* [external/tier4_autoware_msgs](https://github.com/tier4/tier4_autoware_msgs.git)

**NOTE:** Currently, this software is assumed to be built in a separate workspace in order not to contaminate the autoware workspace.
Someday this localizer will be located in the workspace where Autoware blongs. These submodules will be removed at the time.

## Architecture

![node_diagram](docs/node_diagram.png)

### Input topics from sesnors

This localizer requires following topics to work.

|  topic name  |  msg type  | description |
| ---- | ---- | -- |
|  `/sensing/imu/tamagawa/imu_raw`                      |  `sensor_msgs/msg/Imu`                            |  |
|  `/sensing/camera/traffic_light/image_raw/compressed` |  `sensor_msgs/msg/CompressedImage`                |  |
|  `/sensing/camera/traffic_light/camera_info`          |  `sensor_msgs/msg/CameraInfo`                     |  |
|  `/sensing/gnss/ublox/navpvt`                         |  `ublox_msgs/msg/NavPVT`                          | If you use ublox |
|  `/sensing/gnss/septentrio/poscovgeodetic`            |  `septentrio_gnss_driver_msgs/msg/PosCovGeodetic` | If you use Septentrio |
|  `/vehicle/status/velocity_status`                    |  `autoware_auto_vehicle_msgs/msg/VelocityReport`  |  |

### Input topics from autoware

|  topic name  |  msg type  | description |
| ---- | ---- | -- |
|  `/tf_static`      | `tf2_msgs/msg/TFMessage`                   | published from `vehicle_interface` ?  |
|  `/map/vector_map` | `autoware_auto_mapping_msgs/msg/HADMapBin` | published from `/map/lanelet2_map_loader` |

#### about tf_static

* XX1のデータだと `/base_link` から `/traffic_light_left_camera/camera_optical_link` のtf_staticがいる。
  * 厳密にはsubscribeしているcamera_infoの`frame_id`を参照して座標変換している
  * `/sensing/camera/traffic_light` 以外のカメラを使う場合は、それのcamera_infoの`frame_id`へのtf_staticがpublishされている必要がある。
  * `ros2 run tf2_ros tf2_echo base_link traffic_light_left_camera/camera_optical_link`

* 実験用車両とかだと、pilot-auto側でもtf_staticを簡単に流せないことがあるので、回避策を用意している。
* `override_extrinsic`をtrueにすると、undistort_nodeの中でcamera_infoのframe_idを書き換えて、別の外部パラメータを使える。
  * デフォルトでは`override_extrinsic=false`
  * `override_extrinsic=true`だと`impl/imgproc.launch.xml`の中で上書き用のtf_statcがpublishされる

### Output

|  topic name  |  msg type  | description |
| ---- | ---- | -- |
|  `/localicazation/pf/pose`                       | `geometry_msgs/msg/PoseStamped`      | estimated pose  |
|  `/localicazation/validation/overlay_image`      | `sensor_msgs/msg/Image`              | really nice image for demonstration  |
|  `/localicazation/pf/cost_map_image`             | `sensor_msgs/msg/Image`              | visualization of cost map for debug  |
|  `/localicazation/pf/predicted_particles_marker` | `visualization_msgs/msg/MarkerArray` | particles of particle filter |
|  `/localicazation/imgproc/lsd_image`             | `sensor_msgs/msg/Image`              | image |
|  `/localicazation/imgproc/projected_lsd_image`   | `sensor_msgs/msg/Image`              | image |

## How to build

**Supporting `Ubuntu 22.04` + `ROS2 humble` now.
Some branches might support `ROS2 galactic`.**

**NOTE:** Currently, this software is assumed to be built in a separate workspace in order not to contaminate the Autoware workspace.

```bash
mkdir yabloc_ws/src -p
cd yabloc_ws
git clone git@github.com:tier4/YabLoc.git src/YabLoc --recursive
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

<details><summary>The author often use this build command</summary><div>

```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --continue-on-error
```

* (optional) ccache `(--cmake-args) -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache`

* (optional) clang-tidy `(--cmake-args) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

* (optional) test `(--cmake-args) -DBUILD_TESTING=ON`

</div></details>

## Sample data

 [Google drive link](https://drive.google.com/drive/folders/1uNxQ2uPFEGbYXUODQMc7GRO5r9c3Fj6o?usp=share_link)

The link contains *rosbag* and *lanelet2* but *pointcloud*.

## How to execute

**NOTE:** `use_sim_time` is TRUE as default.

### Run with ROSBAG

![how_to_launch_with_rosbag](docs/how_to_launch_with_rosbag.drawio.svg)

```bash
ros2 launch pcdless_launch odaiba_launch.xml standalone:=true
ros2 launch pcdless_launch rviz.launch.xml
ros2 launch autoware_launch logging_simulator.launch.xml \
  system:=false \
  localizaton:=false \
  sensing:=false \
  perception:=false \
  planning:=false \
  control:=false \
  rviz:=false \
  vehicle_model:=jpntaxi \ 
  sensor_model:=aip_xx1 \
  vehicle_id:=5 \
  map_path:=$HOME/Maps/odaiba

ros2 bag play sample_odaiba --clock 100
```

### Run in real world

![how_to_launch_with_rosbag](docs/how_to_launch_in_real.drawio.svg)

**You have to change autoware.universe branch.**

```bash
ros2 launch pcdless_launch odaiba_launch.xml standalone:=true use_sim_time:=false
ros2 launch pcdless_launch rviz.launch.xml
ros2 launch autoware_launch autoware.launch.xml \
  rviz:=false
```

### Run with AWSIM (UNDER CONSTRACTION)

```bash
ros2 launch pcdless_launch odaiba_launch.xml standalone:=true 
ros2 launch pcdless_launch rviz.launch.xml
ros2 launch autoware_launch e2e_simulator.launch.xml
```

## Visualization

(This project contains original rviz plugins.)

![rviz](docs/rviz_description.png)

|  index | topic name | description |
| ---- | ---- | -- |
| 1  |  `/localicazation/validation/overlay_image`     | Projection of lanelet2 (yellow lines) onto image based on estimated pose. If they match well with the actual road markings, it means that the localization performs well.  |
| 2  |  `/localicazation/imgproc/segmented_image`      | result of graph-based segmetation. yellow area is identified as the road surface.|
| 3  |  `/localicazation/pf/cost_map_image`            | cost map generated from lanelet2. |
| 4  |  `/localicazation/imgproc/lsd_image`            | detected line segments |
| 5  |  `/localicazation/map/ground_status`            | ground height and tilt estimatation status |
| 6  |  `/localicazation/twist/kalman/status`          | twist estimation status |
| 7  |  `/localicazation/pf/predicted_particle_marker` | particle distribution of particle fitler (red means a probable candidate) |
| 8  |  `/localicazation/pf/gnss/range_marker`         | particle weight distribution by GNSS |
| 9  |  `/localicazation/pf/scored_cloud`              | 3D projected line segments. the color means the how match they are  |

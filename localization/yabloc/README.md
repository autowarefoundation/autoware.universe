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
Someday this will be located in the workspace where Autoware blongs. These submodules will be removed at the time.

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
|  `/tf_static`      | `tf2_msgs/msg/TFMessage`                   | published from `sensor_kit`  |
|  `/map/vector_map` | `autoware_auto_mapping_msgs/msg/HADMapBin` | published from `/map/lanelet2_map_loader` |

#### about tf_static

Some nodes requires `/tf_static` from `/base_link` to the frame_id of `/sensing/camera/traffic_light/image_raw/compressed` (e.g. `/traffic_light_left_camera/camera_optical_link`).
You can verify that the tf_static is correct with the following command.

```shell
ros2 run tf2_ros tf2_echo base_link traffic_light_left_camera/camera_optical_link
```

If the wrong `/tf_static` are broadcasted because you are using a prototype vehicle, it is useful to give the frame_id in `override_camera_frame_id`.
If you give it a non-empty string, `/imgproc/undistort_node` will rewrite the frame_id in camera_info.
For example, you can give a different tf_static as follows.

```shell
ros2 launch pcdless_launch odaiba_launch.xml override_camera_frame_id:=fake_camera_optical_link
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id fake_camera_optical_link --roll -1.57 --yaw -1.570
```

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

## How to set initialpose

### 1. When YabLoc works `standalone:=true`  (without Autoware's pose_initializer)

1. 2D Pose Estimate in Rviz

You can inidcate x, y and yaw manually in rviz.

2. GNSS Doppler initialization

If doppler (`ublox_msgs/msg/navpvt`) is available and the vehicle moves enough fast, YabLoc will estiamte the initial pose **automatically**.

### 2. When Yabloc works `standalone:=false` (through Autoware's pose_initializer)

<u>UNDER CONSTRUCTION</u>

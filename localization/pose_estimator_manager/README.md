# pose_estimator_manager

Table of contents:

* [Purpose](#purpose)
* [Interface](#interfaces)
* [Architecture](#architecture)
* [How to launch](#how-to-launch)
* [Switching Rules](#switching-rules)

## Purpose

This package launches multiple pose estimators and provides the capability to stop or resume specific pose estimators based on the situation.
The package provides provisional switching rules and will be adaptable to a wide variety of rules in the future.

Please refer to [this discussion](https://github.com/orgs/autowarefoundation/discussions/3878)  about other ideas on implementation.

### Demonstration

The following video demonstrates the switching of four different pose estimators.

TODO: change the video

<div><video controls src="https://user-images.githubusercontent.com/24854875/271473970-eb9f6412-a849-4b12-b487-4bd7aef6de09.mp4" muted="false" width="400"></video></div>

Users can reproduce the demonstration using the following data and launch command:

* rosbag: TODO:
* map: TODO:

```bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=<your-map-path> \
  vehicle_model:=sample_vehicle \
  sensor_model:=awsim_sensor_kit \
  pose_source:=ndt_yabloc_artag_eagleye
```

## Interfaces

### Parameters

| Name                                             | Type   | Description                                                                                 |
|--------------------------------------------------|--------|---------------------------------------------------------------------------------------------|
| `pcd_occupancy_rule/pcd_density_upper_threshold` | double | If the number of occupied voxel around the self-position exceeds this, NDT is allowd        |
| `pcd_occupancy_rule/pcd_density_lower_threshold` | double | If the number of occupied voxel around the self-position is less than this, NDT is disabled |
| `ar_marker_rule/ar_marker_available_distance`    | double | If the distance to the nearest AR marker exceeds this, disable artag-based-localizer        |

### Services

There are no service server.

### Clients

| Name                  | Type                  | Description                       |
|-----------------------|-----------------------|-----------------------------------|
| `/yabloc_suspend_srv` | std_srv::srv::SetBool | service to stop or restart yabloc |

### Subscriptions

For pose estimator arbitration:

| Name                                  | Type                                          | Description    |
|---------------------------------------|-----------------------------------------------|----------------|
| `/input/artag/image`                  | sensor_msgs::msg::Image                       | ArTag input    |
| `/input/yabloc/image`                 | sensor_msgs::msg::Image                       | YabLoc input    |
| `/input/eagleye/pose_with_covariance` | geometry_msgs::msg::PoseWithCovarianceStamped | Eagleye output |
| `/input/ndt/pointcloud`               | sensor_msgs::msg::PointCloud2                 | NDT input|

For swithing rule:

| Name                          | Type                                                         | Description                       |
|-------------------------------|--------------------------------------------------------------|-----------------------------------|
| `/input/pointcloud_map`       | sensor_msgs::msg::PointCloud2                                | point cloud map                   |
| `/input/vector_map`           | autoware_auto_mapping_msgs::msg::HADMapBin                   | vector map                        |
| `/input/pose_with_covariance` | geometry_msgs::msg::PoseWithCovarianceStamped                | localization final output         |
| `/input/initialization_state` | autoware_adapi_v1_msgs::msg::LocalizationInitializationState | localization initialization state |

### Publications

| Name                                   | Type                                          | Description                                            |
|----------------------------------------|-----------------------------------------------|--------------------------------------------------------|
| `/output/artag/image`                  | sensor_msgs::msg::Image                       | relayed ArTag input                                    |
| `/output/yabloc/image`                 | sensor_msgs::msg::Image                       | relayed YabLoc input                                   |
| `/output/eagleye/pose_with_covariance` | geometry_msgs::msg::PoseWithCovarianceStamped | relayed Eagleye output                                 |
| `/output/ndt/pointcloud`               | sensor_msgs::msg::PointCloud2                 | relayed NDT input                                      |
| `/output/debug/marker_array`           | visualization_msgs::msg::MarkerArray          | [debug topic] everything for visualization             |
| `/output/debug/string`                 | visualization_msgs::msg::MarkerArray          | [debug topic] debug information such as current status |


## Architecture

### Case of running a single pose estimator

<img src="./media/single_pose_estimator.drawio.svg" alt="drawing" width="600"/>


### Case of running multiple pose estimators

<img src="./media/architecture.drawio.svg" alt="drawing" width="800"/>


## How to launch

The user can launch the desired pose_estimators by giving the pose_estimator names as a concatenation of underscores for the runtime argument `pose_source`.

```bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=<your-map-path> \
  vehicle_model:=sample_vehicle \
  sensor_model:=awsim_sensor_kit \
  pose_source:=ndt_yabloc_artag_eagleye
```

Even if `pose_source` includes an unexpected string, it will be filtered appropriately.
Please see the table below for details.

| given runtime argument                       | parsed pose_estimator_manager's param (pose_sources) |
|----------------------------------------------|------------------------------------------------------|
| `pose_source:=ndt`                           | `["ndt"]`                                            |
| `pose_source:=hoge`                          | `[]`                                                 |
| `pose_source:=yabloc_ndt`                    | `["ndt","yabloc"]`                                   |
| `pose_source:=yabloc_ndt_ndt_ndt`            | `["ndt","yabloc"]`                                   |
| `pose_source:=ndt_yabloc_eagleye`            | `["ndt","yabloc","eagleye"]`                         |
| `pose_source:=ndt_yabloc_hoge_eagleye_artag` | `["ndt","yabloc","eagleye","artag"]`                 |


## Switching Rules

Currently, only one rule (map based rule) is implemented, but in the future, multiple rules will be implemented.

### Map Based Rule


```mermaid
flowchart LR
  A{<1>\nLocalization\n Initialization\n state is 'INITIALIZED'?}
  A --false --> _A[enable all]
  A --true --> B{<2>\n/localization\n/pose_with_covariance\n is subscribed?}
  B -- false --> _B[enable all]
  B -- true --> C{<3>\neagleye is\navailable}
  C -- true --> _C[enable Eagleye]
  C -- false --> D[#]

  D'[#] --> D''{<4>\nArTag is\navailable}
  D'' -- false --> _D[enable ArTag]
  D'' -- true --> E{<5>\nyabloc is\navailable}
  E -- false --> _E[enable NDT]
  E -- true --> F{<6>\nNDT is\navailable}
  F -- false --> _F[enable YabLoc]
  F -- true --> G{<7>\nNDT is more suitable\nthan yabloc}
  G -- true --> _G[enable NDT]
  G -- false --> __G[enable YabLoc]
```

In the flowchart, any pose_estimators which are not enabled are disabled.
This rule basically allows only one pose_estimator to be activated.

| branch index | description                                                             |
|--------|-------------------------------------------------------------------------|
| 1    | If localization initialization state is not `INITIALIZED`, enable all pose_estimators. This is because system does not know which pose_estimator is  available for initial localization.|
| 2    | If localization initialization state is not `INITIALIZED`, enable all pose_estimators. This is bacause it is not possible to determine which pose_estimators are available due to the current position.|
| 3    | 自己位置がEagleye areaに含まれている場合、Eagleyeを有効にする。eagleye areaの詳細は[Eagleye area](#eagleye-area)を見て。         |
| 4    | ARタグ用のランドマークが周辺にある場合、ARマーカーベース位置推定を有効にする。         |
| 5    | YabLocが実行時引数で有効にされていない場合、NDTを起動する。         |
| 6    | NDTが実行時引数で有効にされていない場合、YabLocを起動する。         |
| 7    | PCD occupancyがしきい値を上回っていればNDTを有効にする。 PCD occupancyの詳細は[PCD occupancy](#pcd-occupancy)を見て。         |


## Rule helpers

* [PCD occupancy](#pcd-occupancy)
* [Eagleye area](#eagleye-area)
* [AR tag position](#ar-tag-position)

### PCD occupancy 

<img src="./media/pcd_occupancy.drawio.svg" alt="drawing" width="800"/>

### eagleye area

The values provided below are placeholders. Ensure to input the correct coordinates corresponding to the actual location where the area is specified, such as lat, lon, mgrs_code, local_x, local_y.
The following snipet is an example of eagleye area.

```xml
  <node id="1" lat="35.8xxxxx" lon="139.6xxxxx">
    <tag k="mgrs_code" v="54SUE000000"/>
    <tag k="local_x" v="10.0"/>
    <tag k="local_y" v="10.0"/>
    <tag k="ele" v="1.0"/>
  </node>
  <node id="2" lat="35.8xxxxx" lon="139.6xxxxx">
    <tag k="mgrs_code" v="54SUE000000"/>
    <tag k="local_x" v="10.0"/>
    <tag k="local_y" v="20.0"/>
    <tag k="ele" v="1.0"/>
  </node>
  <node id="3" lat="35.8xxxxx" lon="139.6xxxxx">
    <tag k="mgrs_code" v="54SUE000000"/>
    <tag k="local_x" v="20.0"/>
    <tag k="local_y" v="20.0"/>
    <tag k="ele" v="1.0"/>
  </node>
  <node id="4" lat="35.8xxxxx" lon="139.6xxxxx">
    <tag k="mgrs_code" v="54SUE000000"/>
    <tag k="local_x" v="10.0"/>
    <tag k="local_y" v="20.0"/>
    <tag k="ele" v="1.0"/>
  </node>

...

  <way id="5">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="type" v="eagleye_area"/>
    <tag k="area" v="yes"/>
  </way>

```

### AR tag position



## For developers


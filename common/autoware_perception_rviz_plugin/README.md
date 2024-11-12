# autoware_perception_rviz_plugin

## 目的

認識モジュールからの結果を可視化するためのrvizプラグインです。このパッケージは、Autoware.Autoによって開発されたrvizプラグインの実装をベースにしています。

元の設計思想については、Autoware.Auto設計ドキュメントを参照してください。[[1]](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/tools/visualization/autoware_rviz_plugins)

<!-- このパッケージの目的を記述し、機能を簡単に説明してください。

例:
  {package_name} は、障害物を回避できる経路を計画するためのパッケージです。
  この機能は、障害物のフィルタリングと経路の最適化という2つのステップで構成されています。
-->

## 入力タイプ / 可視化結果

### 検出された物体

#### 入力タイプ
- `AutowareAuto::DetectedObject`

#### Visualized Result
- `rviz::MarkerArray`

### LandMarks

#### Input Types
- `AutowareAuto::LandMark`

#### Visualized Result
- `rviz::MarkerArray`

### PointClouds

#### Input Types
- `AutowareAuto::PointCloud`
- `AutowareAuto::LidarPointCloud`
- `AutowareAuto::CameraPointCloud`
- `AutowareAuto::PointCloud2`

#### Visualized Result
- `sensor_msgs::PointCloud2`

### TrackingObjects

#### Input Types
- `AutowareAuto::TrackedObject`

#### Visualized Result
- `rviz::MarkerArray`

### TrafficLights

#### Input Types
- `AutowareAuto::TrafficLight`

#### Visualized Result
- `rviz::MarkerArray`

### Planning (optional)

#### Input Types
- `AutowareAuto::Trajectory`

#### Visualized Result
- `rviz::MarkerArray`

## Configuration

### AutowareAuto::DetectedObjects

- `Topic Name`: The topic name of the `AutowareAuto::DetectedObject` message received by the plugin.

- `Marker Scale`: The scale of the marker.

- `Marker Lifetime`: The lifetime of the marker in seconds.

- `Marker Alpha`: The alpha value of the marker.

- `Marker Color`: The color of the marker.

- `Ground Truth Marker Enabled`: Enable/disable the display of ground truth marker.

- `Measured Marker Enabled`: Enable/disable the display of measured marker.

- `Ground Truth Color`: The color of the ground truth marker.

- `Measured Color`: The color of the measured marker.

### LandMarks

- `Topic Name`: The topic name of the `AutowareAuto::LandMark` message received by the plugin.

- `Marker Scale`: The scale of the marker.

- `Marker Lifetime`: The lifetime of the marker in seconds.

- `Marker Alpha`: The alpha value of the marker.

- `Marker Color`: The color of the marker.

### PointClouds

- `Topic Name`: The topic name of the `AutowareAuto::PointCloud` message received by the plugin.

- `Show Color`: Enable/disable the display of the point cloud with color information.

- `Color Attribute`: The attribute name of the color information.

- `Normalize Color`: Enable/disable the normalization of the color information.

- `Min Intensity`: The minimum intensity threshold for displaying the point cloud.

- `Max Intensity`: The maximum intensity threshold for displaying the point cloud.

### TrackingObjects

- `Topic Name`: The topic name of the `AutowareAuto::TrackedObject` message received by the plugin.

- `Marker Scale`: The scale of the marker.

- `Marker Lifetime`: The lifetime of the marker in seconds.

- `Marker Alpha`: The alpha value of the marker.

- `Marker Color`: The color of the marker.

### TrafficLights

- `Topic Name`: The topic name of the `AutowareAuto::TrafficLight` message received by the plugin.

- `Marker Scale`: The scale of the marker.

- `Marker Lifetime`: The lifetime of the marker in seconds.

- `Marker Alpha`: The alpha value of the marker.

- `Marker Color`: The color of the marker.

### Planning

- `Topic Name`: The topic name of the `AutowareAuto::Trajectory` message received by the plugin.

- `Marker Scale`: The scale of the marker.

- `Marker Lifetime`: The lifetime of the marker in seconds.

- `Marker Alpha`: The alpha value of the marker.

- `Marker Color`: The color of the marker.
- `Show Reference Trajectory`: Enable/disable the display of the reference trajectory.
- `Show Prediction Trajectory`: Enable/disable the display of the prediction trajectory.
- `Show Planning Trajectory`: Enable/disable the display of the planning trajectory.
- `Show Predicted Obstacle Trajectory`: Enable/disable the display of the predicted obstacle trajectory.
- `Show Sensor Predicted Obstacle`: Enable/disable the display of the sensor predicted obstacle.
- `Show Speed Profile`: Enable/disable the display of the speed profile.
- `Show Lateral Speed Profile`: Enable/disable the display of the lateral speed profile.
- `Show Longitudinal Speed Profile`: Enable/disable the display of the longitudinal speed profile.
- `Show Selected Motion Plan`: Enable/disable the display of the selected motion plan.
- `Show Goal Pose`: Enable/disable the display of the goal pose.
- `Prediction Trajectory Duration`: The duration of the prediction trajectory in seconds.
- `Planning Trajectory Duration`: The duration of the planning trajectory in seconds.
- `Reference Trajectory Duration`: The duration of the reference trajectory in seconds.
- `Reference Velocity`: The velocity of the reference trajectory in meters per second.
- `Reference Acceleration`: The acceleration of the reference trajectory in meters per second squared.
- `Reference Jerk`: The jerk of the reference trajectory in meters per second cubed.
- `Max Velocity Violation (m/s)`: The maximum velocity逸脱量 in meters per second.
- `Max Lateral Acceleration Violation (m/s^2)`: The maximum lateral acceleration逸脱量 in meters per second squared.
- `Max Longitudinal Acceleration Violation (m/s^2)`: The maximum longitudinal acceleration逸脱量 in meters per second squared.
- `Max Jerk Violation (m/s^3)`: The maximum jerk逸脱量 in meters per second cubed.
- `Max Curvature Violation (1/m)`: The maximum curvature逸脱量 in inverse meters.
- `Additional Moving Obstacle`: The path of an additional moving obstacle for planning.
- `Initial Distance to Moving Obstacle`: The initial distance to the moving obstacle in meters.
- `Perception Planning Horizon`: The perception planning horizon.
- `Sensor Planning Horizon`: The sensor planning horizon.
- `Topic Name`: The topic name of the selected motion plan.

### Planned Path Optimization (optional)

- `Topic Name`: The topic name of the `autoware_planning_msgs::PlannedPathOptimization` message received by the plugin.

- `Marker Scale`: The scale of the marker.

- `Marker Lifetime`: The lifetime of the marker in seconds.

- `Marker Alpha`: The alpha value of the marker.

- `Marker Color`: The color of the marker.

- `Optimization Success Marker Color`: The color of the optimization success marker.

- `Optimization Failure Marker Color`: The color of the optimization failure marker.

## Usage

```
rviz
```

```
roslaunch autoware_perception_rviz_plugin autoware_perception_rviz_plugin.launch
```

| 名称 | タイプ                                             | 説明            |
| ---------- | ------------------------------------------------ | ---------------------- |
|      | `autoware_perception_msgs::msg::DetectedObjects` | 検出結果配列 |

#### 可視化結果

![detected-object-visualization-description](./images/detected-object-visualization-description.jpg)

### トラッキングされたオブジェクト

#### 入力タイプ

| 名称 | タイプ | 説明 |
|---|---|---|
| | `autoware_perception_msgs::msg::TrackedObjects` | 追跡結果の配列 |

#### 可視化結果

検出結果で追跡結果を上書きします。

![tracked-object-visualization-description](./images/tracked-object-visualization-description.jpg)

### PredictedObjects

#### 入力タイプ

| 名前 | 種別                                             | 説明                     |
| ---- | ------------------------------------------------- | ------------------------- |
|      | `autoware_perception_msgs::msg::PredictedObjects` | prediction結果配列       |

#### 可視化結果

追跡結果を予測結果で上書きします。

![predicted-object-visualization-description](./images/predicted-object-visualization-description.jpg)

## 参照/外部リンク

[1] <https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/tools/visualization/autoware_rviz_plugins>

## 今後の拡張/未実装部分


## `traffic_light_multi_camera_fusion` パッケージ

### 概要

`traffic_light_multi_camera_fusion` は、以下の 2 つのタスクにまとめられる、交通信号の融合を行います。

1. 多眼カメラ融合: 別々のカメラで検出された単一の交通信号で行われます。
2. グループ融合: 同一群内の交通信号で行われます。これには、lanelet2 マップで定義された同じ規制要素 ID を共有する信号が含まれます。

### 入力トピック

各カメラについて、以下の 3 つのトピックがサブスクライブされます:

| 名称                                    | 種類                                           | 説明                                               |
| -------------------------------------- | ---------------------------------------------- | ---------------------------------------------------- |
| `~/<camera_namespace>/camera_info`     | `sensor_msgs::CameraInfo`                        | `traffic_light_map_based_detector`からのカメラ情報 |
| `~/<camera_namespace>/rois`            | `tier4_perception_msgs::TrafficLightRoiArray`    | `traffic_light_fine_detector`からの検出ROI        |
| `~/<camera_namespace>/traffic_signals` | `tier4_perception_msgs::TrafficLightSignalArray` | `traffic_light_classifier`からの分類結果          |

これらのトピックを手動で設定する必要はありません。`camera_namespaces` パラメータを提供するだけで、ノードは `<camera_namespace>` を自動的に抽出し、サブスクライバーを作成します。

## 出力トピック

| 名                          | タイプ                                                 | 説明                                 |
| --------------------------- | ------------------------------------------------------ | ------------------------------------- |
| `~/output/traffic_signals` | autoware_perception_msgs::TrafficLightSignalArray | 交通信号の融合結果                   |

## ノードパラメータ

---

### Dynamic obstacles trajectory predictor (Dynamic tracker)
  - Name: `~dynamic_tracker/tracking_time_window`
    - Type: Float
    - Default value: 4.0
    - Description: オブスタクルを追跡するための時間窓 (秒)

  - Name: `~dynamic_tracker/min_time_gap`
    - Type: Float
    - Default value: 1.0
    - Description: Dynamic Planning によって生成される経路上で、オブジェクトが追跡されるために必要な最小のタイムギャップ (秒)

  - Name: `~dynamic_tracker/max_time_gap`
    - Type: Float
    - Default value: 2.0
    - Description: Dynamic Planning によって生成される経路上で、オブジェクトが追跡されるために必要な最大タイムギャップ (秒)

  - Name: `~dynamic_tracker/acceleration_v2`
    - Type: Float
    - Default value: 2.0
    - Description: オブジェクトの加速度の最大値 (m/s^2)

  - Name: `~dynamic_tracker/longitudinal_jerk_v2`
    - Type: Float
    - Default value: 4.0
    - Description: オブジェクトの縦方向ジャークの最大値 (m/s^3)

  - Name: `~dynamic_tracker/lateral_jerk_v2`
    - Type: Float
    - Default value: 8.0
    - Description: オブジェクトの横方向ジャークの最大値 (m/s^3)

  - Name: `~dynamic_tracker/use_v_and_a`
    - Type: Bool
    - Default value: False
    - Description: 速度と加速度を予測に使用するかどうか

  - Name: `~dynamic_tracker/consider_turning_radius`
    - Type: Bool
    - Default value: True
    - Description: 予測の際、オブジェクトの旋回半径を考慮するかどうか

  - Name: `~dynamic_tracker/max_obstacle_age`
    - Type: Float
    - Default value: 1.0
    - Description: オブジェクトが追跡されるために必要な最大年齢 (秒)

  - Name: `~dynamic_tracker/pcd_post_resampling`
    - Type: Bool
    - Default value: False
    - Description: `post resampling`後に点群を使用するかどうか

  - Name: `~dynamic_tracker/estimate_past_pose`
    - Type: Bool
    - Default value: True
    - Description: オブジェクトの過去の位置を推定するかどうか

  - Name: `~dynamic_tracker/acceleration_lin_deceleration_v2`
    - Type: Float
    - Default value: 2.0
    - Description: オブジェクトの直線減速度の最大値 (m/s^2)

### Obstacle slice planner
  - Name: `~obstacle_slice_planner/enable_rollout_detection`
    - Type: Bool
    - Default value: True
    - Description: ロールアウト検出を有効にするかどうか

  - Name: `~obstacle_slice_planner/rollout_detection_min_lane_idx`
    - Type: Int
    - Default value: 1
    - Description: レーンインデックスの最小値。これ未満のレーンのオブジェクトはロールアウト検出の対象外

  - Name: `~obstacle_slice_planner/rollout_detection_max_lane_idx`
    - Type: Int
    - Default value: 2
    - Description: レーンインデックスの最大値。これ以上のレーンのオブジェクトはロールアウト検出の対象外

  - Name: `~obstacle_slice_planner/rollout_detection_min_ego_v`
    - Type: Float
    - Default value: 0.1
    - Description: ロールアウト検出に使用する、自車速度の最小値 (m/s)

  - Name: `~obstacle_slice_planner/rollout_detection_min_v_diff`
    - Type: Float
    - Default value: 0.5
    - Description: ロールアウト検出に使用する、自車と対象オブジェクトの速度差の最小値 (m/s)

  - Name: `~obstacle_slice_planner/rollout_detection_horizontal_distance`
    - Type: Float
    - Default value: 15.0
    - Description: ロールアウト検出に使用する、自車と対象オブジェクトの水平距離の最大値 (m)

  - Name: `~obstacle_slice_planner/rollout_detection_time_offset`
    - Type: Float
    - Default value: 3.0
    - Description: ロールアウト検出に使用する、タイムオフセットの最大値 (秒)

  - Name: `~obstacle_slice_planner/rollout_detection_speed_limit`
    - Type: Float
    - Default value: 20.0
    - Description: ロールアウト検出に使用する、道路の速度制限 (m/s)

  - Name: `~obstacle_slice_planner/rollout_detection_lateral_interval`
    - Type: Float
    - Default value: 2.0
    - Description: ロールアウト検出に使用する、横方向のインターバル (m)

  - Name: `~obstacle_slice_planner/rollout_detection_grid_resolution`
    - Type: Float
    - Default value: 1.0
    - Description: ロールアウト検出に使用する、グリッド解像度 (m)

  - Name: `~obstacle_slice_planner/rollout_detection_overlap_threshold`
    - Type: Float
    - Default value: 0.5
    - Description: ロールアウト検出に使用する、オブジェクトの重なり量のしきい値

  - Name: `~obstacle_slice_planner/rollout_detection_angle_diff_threshold`
    - Type: Float
    - Default value: 20.0
    - Description: ロールアウト検出に使用する、オブジェクトの方向角差のしきい値 (deg)

  - Name: `~obstacle_slice_planner/candidate_index_list`
    - Type: IntList
    - Default value: []
    - Description: 候補となるスライスインデックスのリスト

  - Name: `~obstacle_slice_planner/candidate_pose_list`
    - Type: PoseArray
    - Default value: []
    - Description: 候補となる自車位置のリスト

### Motion planner
  - Name: `~motion_planner/sample_interval`
    - Type: Float
    - Default value: 0.5
    - Description: サンプリングの間隔 (秒)

  - Name: `~motion_planner/max_sample_number`
    - Type: Int
    - Default value: 1000
    - Description: 最大サンプリング数

  - Name: `~motion_planner/max_calc_time`
    - Type: Float
    - Default value: 0.1
    - Description: Motion Planning の最大計算時間 (秒)

  - Name: `~motion_planner/speed_limit`
    - Type: Float
    - Default value: 20.0
    - Description: 経路生成時の速度制限 (m/s)

  - Name: `~motion_planner/max_acceleration`
    - Type: Float
    - Default value: 2.0
    - Description: 経路生成時の最大加速度 (m/s^2)

  - Name: `~motion_planner/max_lateral_jerk`
    - Type: Float
    - Default value: 4.0
    - Description: 経路生成時の最大横方向ジャーク (m/s^3)

  - Name: `~motion_planner/max_lateral_acceleration`
    - Type: Float
    - Default value: 2.0
    - Description: 経路生成時の最大横方向加速度 (m/s^2)

  - Name: `~motion_planner/target_speed`
    - Type: Float
    - Default value: 10.0
    - Description: Motion Planning のターゲット速度 (m/s)

  - Name: `~motion_planner/min_speed`
    - Type: Float
    - Default value: 2.0
    - Description: Motion Planning の最小速度 (m/s)

  - Name: `~motion_planner/enable_smoothed_path`
    - Type: Bool
    - Default value: True
    - Description: Smooth Path Planning を有効にするかどうか

  - Name: `~motion_planner/smoothed_path_epsilon`
    - Type: Float
    - Default value: 0.1
    - Description: Smooth Path Planning のイプシロン値

  - Name: `~motion_planner/curvature_threshold`
    - Type: Float
    - Default value

| パラメータ                  | 型             | 説明                                         |
| --------------------------- | ------------- | -------------------------------------------- |
| `camera_namespaces`       | vector<string> | 融合されるカメラのネームスペース              |
| `message_lifespan`        | double         | 融合されるタイムスタンプの最大スパン         |
| `approximate_sync`        | bool           | 近似同期モードで動作するかどうか              |
| `perform_group_fusion`    | bool           | グループ融合を実行するかどうか                  |


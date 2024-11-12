# Behavior Velocity Planner

## 概要

`behavior_velocity_planner` は交通規則に基づいて速度を調整するプランナーです。
モジュールをプラグインとして読み込みます。各モジュールについて詳しくは、次のリンクを参照してください。

![アーキテクチャ](./docs/BehaviorVelocityPlanner-Architecture.drawio.svg)

- [Blind Spot](../autoware_behavior_velocity_blind_spot_module/README.md)
- [Crosswalk](../autoware_behavior_velocity_crosswalk_module/README.md)
- [Walkway](../autoware_behavior_velocity_walkway_module/README.md)
- [Detection Area](../autoware_behavior_velocity_detection_area_module/README.md)
- [Intersection](../autoware_behavior_velocity_intersection_module/README.md)
- [MergeFromPrivate](../autoware_behavior_velocity_intersection_module/README.md#merge-from-private)
- [Stop Line](../autoware_behavior_velocity_stop_line_module/README.md)
- [Virtual Traffic Light](../autoware_behavior_velocity_virtual_traffic_light_module/README.md)
- [Traffic Light](../autoware_behavior_velocity_traffic_light_module/README.md)
- [Occlusion Spot](../autoware_behavior_velocity_occlusion_spot_module/README.md)
- [No Stopping Area](../autoware_behavior_velocity_no_stopping_area_module/README.md)
- [Run Out](../autoware_behavior_velocity_run_out_module/README.md)
- [Speed Bump](../autoware_behavior_velocity_speed_bump_module/README.md)

各モジュールが速度を計画する際は、`base_link`（後車軸の中央）の姿勢を基準として考慮します。
たとえば、ストップラインに車の前面を向けて停止するには、`base_link`から前面までの距離から`base_link`の姿勢を計算し、`base_link`の姿勢からパス速度を変更します。

![set_stop_velocity](./docs/set_stop_velocity.drawio.svg)

## 入力トピック

| 名前                                    | タイプ                                                  | 説明                                                                                                                     |
| --------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `~input/path_with_lane_id`              | tier4_planning_msgs::msg::PathWithLaneId              | レーンID付きパス                                                                                                               |
| `~input/vector_map`                     | autoware_map_msgs::msg::LaneletMapBin                 | ベクターマップ                                                                                                                      |
| `~input/vehicle_odometry`               | nav_msgs::msg::Odometry                               | 車両の速度                                                                                                                |
| `~input/dynamic_objects`                | autoware_perception_msgs::msg::PredictedObjects       | 動的オブジェクト                                                                                                                 |
| `~input/no_ground_pointcloud`           | sensor_msgs::msg::PointCloud2                         | 障害物点群                                                                                                             |
| `~/input/compare_map_filtered_pointcloud` | sensor_msgs::msg::PointCloud2                         | 比較マップでフィルタリングされた障害物点群（この実行モジュールの検出方法がPointsの場合に使用されます） |
| `~input/traffic_signals`                | autoware_perception_msgs::msg::TrafficLightGroupArray | 信号状態                                                                                                            |

## 出力トピック

| 名前                   | 型                                      | 説明                             |
| ---------------------- | ----------------------------------------- | ----------------------------------- |
| `~output/path`         | `autoware_planning_msgs::msg::Path`         | 走行すべきパス                     |
| `~output/stop_reasons` | `tier4_planning_msgs::msg::StopReasonArray` | 車両を停止させる理由               |

## ノードパラメータ

| パラメーター        | タイプ            | 説明                                                                                                           |
| ------------------- | ----------------- | ---------------------------------------------------------------------------------------------------------------- |
| `launch_modules`     | 文字列のベクトル | 起動するモジュール名                                                                                            |
| `forward_path_length` | double           | フォワードパス長                                                                                                |
| `backward_path_length` | double           | バックワードパス長                                                                                               |
| `max_accel`          | double           | (グローバルパラメータ) 車両の最大加速度                                                                           |
| `system_delay`       | double           | (グローバルパラメータ) 制御コマンドの出力を始めるまでの遅延時間                                                   |
| `delay_response_time` | double           | (グローバルパラメータ) 車両が制御コマンドに応答するまでの遅延時間                                               |

## シミュレーション/実環境における信号機処理

信号機情報の処理は用途によって異なります。以下の表では、対応する車線の信号機トピック要素を `info` とし、`info` が使用できない場合は `null` とします。

| モジュール/ケース                                                                                                 | `info` が `null`                | `info` が `null` 以外                                                                                                                                                                                       |
| :------------------------------------------------------------------------------------------------------------- | ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| intersection_occlusion(`is_simulation = *`) <ul> <li>`info` は最新の非 `null` 情報</li></ul>                | GO (遮蔽は無視されます)         | intersection_occlusion は現在までのキューの中で最新の UNKNOWN 以外の情報を利用します。<ul><li>`info` が `GREEN` または `UNKNOWN` の場合、遮蔽が考慮されます</li><li>`info` が `RED` または `YELLOW` の場合、遮蔽は無視されます (GO)</ li> <li> 注: 現在、タイムアウトは考慮されていません</li> </ul> |
| traffic_light(sim, `is_simulation = true`) <ul> <li>`info` は現在の情報</li></ul>                       | GO                            | traffic_light は現在知覚している交通信号情報を直接使用します。<ul><li>`info` がタイムアウトの場合は、色に関係なく停止します</li> <li>`info` がタイムアウトでない場合は、色に応じて動作します。`info` が `UNKNOWN` の場合は、停止します</li></ul> {: rowspan=2} |
| traffic_light(real, `is_simulation = false`) <ul> <li>`info` は現在の情報</li></ul>                      | 停止                           | {: style="padding:0"}                                                                                                                                                                                                                                                              |
| 歩行者横断歩道と交通信号 (`is_simulation = *`) <ul> <li>`info` は現在の情報</li></ul>                   | デフォルト                     | <ul> <li>`disable_yield_for_new_stopped_object` が true の場合、各サブ scene_module はモジュールインスタンス化後に検出された新しい歩行者を無視します。</li> <li>`ignore_with_traffic_light` が true の場合、遮蔽検出はスキップされます。</li></ul> |
| map_based_prediction(`is_simulation = *`) <ul> <li>`info` は現在の情報</li></ul>                        | デフォルト                     | 歩行者信号が<ul> <li>RED の場合、周囲の歩行者は予測されません。</li> <li>GREEN の場合、停止中の歩行者は予測されません。</li></ul>                                                                                                                        |


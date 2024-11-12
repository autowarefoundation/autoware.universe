# 運動速度プランナー

## 概要

`motion_velocity_planner`は、車両周囲の障害物に基づいて経路速度を調整するためのプランナーです。
プラグインとしてモジュールを読み込みます。各モジュールの詳細については、以下にリストされているリンクを参照してください。

![アーキテクチャ](./docs/MotionVelocityPlanner-InternalInterface.drawio.svg)

- [Out of Lane](../autoware_motion_velocity_out_of_lane_module/README.md)

各モジュールは自車トラジェクトリに挿入される停止および減速点を計算します。
これらの点は、自車がトラジェクトリに従う場合の自車`base_link`フレームに対応すると想定されています。
つまり、壁の前に停止するには、車体前オフセット（ホイールベース + 前方オーバーハング、[車両寸法](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-dimensions/)を参照）に等しい距離でトラジェクトリに停止点が挿入されます。

![停止速度を設定](./docs/set_stop_velocity.drawio.svg)

## 入力トピック

| 名称                                | 型                                              | 説明                       |
| ------------------------------- | ----------------------------------------------- | ---------------------------- |
| `~/input/trajectory`             | `autoware_planning_msgs::msg::Trajectory`     | 入力軌道                   |
| `~/input/vector_map`             | `autoware_map_msgs::msg::LaneletMapBin`         | ベクターマップ               |
| `~/input/vehicle_odometry`       | `nav_msgs::msg::Odometry`                      | 自車位置と速度               |
| `~/input/accel`                  | `geometry_msgs::msg::AccelWithCovarianceStamped` | 自車加速度                 |
| `~/input/dynamic_objects`        | `autoware_perception_msgs::msg::PredictedObjects` | 動的オブジェクト             |
| `~/input/no_ground_pointcloud`   | `sensor_msgs::msg::PointCloud2`                 | 障害物点群                 |
| `~/input/traffic_signals`        | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 交通信号状態               |
| `~/input/virtual_traffic_light_states` | `tier4_v2x_msgs::msg::VirtualTrafficLightStateArray` | 仮想交通信号状態           |
| `~/input/occupancy_grid`         | `nav_msgs::msg::OccupancyGrid`                  | オキュパンシグリッド         |

## 出力トピック

| 名称 | タイプ | 説明 |
|---|---|---|
| `~/output/trajectory` | `autoware_planning_msgs::msg::Trajectory` | 速度プロファイルを更新したエゴトラジェクトリ |
| `~/output/velocity_factors` | `autoware_adapi_v1_msgs::msg::VelocityFactorsArray` | エゴの速度プロファイルを変化させる要因 |

## サービス

### Planning
- self-localization: `current pose` の更新
- behavior-planning: `trajectory` の生成
- trajectory-planning: `trajectory` の追従制御
- perception: 感知データの提供

### Control
- steering-control: ステアリング制御
- braking-control: ブレーキ制御
- throttle-control: アクセル制御

### Mapping
- map: 地図データの提供

### Monitoring
- system-monitoring: システム状態の監視

### Visualization
- visualization: 各種情報の可視化

| 名前 | 種類 | 説明 |
|---|---|---|
| `~/service/load_plugin` | autoware_motion_velocity_planner_node::srv::LoadPlugin | プラグインのロードをリクエスト |
| `~/service/unload_plugin` | autoware_motion_velocity_planner_node::srv::UnloadPlugin | プラグインのアンロードをリクエスト |

## ノードパラメータ

| パラメータ        | 型             | 説明            |
| ---------------- | ---------------- | ---------------------- |
| `launch_modules` | vector\<string\> | 起動するモジュールの名前 |

加えて、以下のパラメータをノードに提供する必要があります。

- [最近接探索パラメータ](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/nearest_search.param.yaml)
- [車両情報パラメータ](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_description/config/vehicle_info.param.yaml)
- [一般的なPlanningパラメータ](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/common.param.yaml)
- [Smootherパラメータ](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_velocity_smoother/#parameters)
- ロードされる各プラグインのパラメータ


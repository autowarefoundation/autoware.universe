# autoware_overlay_rviz_plugin

RViz2の3Dシーン上に2Dオーバーレイを表示するためのプラグイン。

[jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)
パッケージをベースとしており、3条項BSDライセンスに従っています。

## 目的

このプラグインは、車両速度、ウィンカー、ステアリング状態、ギアの視覚的かつわかりやすい表示を提供します。

## 入出力

### 入力

| 名前                                                 | 種類                                                      | 説明                                         |
| -------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------- |
| `/vehicle/status/velocity_status`                  | `autoware_vehicle_msgs::msg::VelocityReport`          | 車両速度のトピック                                  |
| `/vehicle/status/turn_indicators_status`           | `autoware_vehicle_msgs::msg::TurnIndicatorsReport`     | ウインカーの状態のトピック                             |
| `/vehicle/status/hazard_status`                     | `autoware_vehicle_msgs::msg::HazardReport`             | ハザードランプの状態のトピック                          |
| `/vehicle/status/steering_status`                  | `autoware_vehicle_msgs::msg::SteeringReport`           | ステアリングの状態のトピック                               |
| `/vehicle/status/gear_status`                      | `autoware_vehicle_msgs::msg::GearReport`               | ギアのステータスに関するトピック                              |
| `/planning/scenario_planning/current_max_velocity`   | `tier4_planning_msgs::msg::VelocityLimit`              | 速度制限に関するトピック                                    |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 信号機のステータスに関するトピック                          |

## パラメータ

### 中核パラメータ

#### SignalDisplay

| 名称                     | タイプ | デフォルト値        | 説明                       |
| ------------------------ | ------ | -------------------- | --------------------------------- |
| `property_width_`        | int    | 128                  | プロッターウィンドウの幅 [px]  |
| `property_height_`       | int    | 128                  | プロッターウィンドウの高さ [px] |
| `property_left_`         | int    | 128                  | プロッターウィンドウの左 [px]   |
| `property_top_`          | int    | 128                  | プロッターウィンドウの上 [px]    |
| `property_signal_color_` | QColor | QColor(25, 255, 240) | ターンシグナルの色                 |

## 前提条件 / 制約

TBD.

## 使用法

1. `rviz2`を起動し、`Displays`パネルの`追加`ボタンをクリックします。

   ![select_add](./assets/images/select_add.png)

2. `By display type`タブで、`autoware_overlay_rviz_plugin/SignalDisplay`を選択し、OKを押します。

3. 必要に応じて、トピックの名前を入力します。

   ![select_topic_name](./assets/images/select_topic_name.png)


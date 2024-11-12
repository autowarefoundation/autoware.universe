## Tier4 vehicle RVIZ プラグイン

このパッケージには jsk コードが含まれています。
jsk_overlay_utils.cpp および jsk_overlay_utils.hpp は BSD ライセンスであることに注意してください。

## 目的

このプラグインは、車両速度、ターンシグナル、ステアリングステータス、加速度をビジュアルかつ分かりやすく表示します。

## 入出力

### 入力

| 名前                                | タイプ                                                   | 説明                                 |
| ------------------------------------ | ---------------------------------------------------------- | -------------------------------------- |
| `/vehicle/status/velocity_status`    | `autoware_vehicle_msgs::msg::VelocityReport`                 | 車両の捻じれ                             |
| `/control/turn_signal_cmd`           | `autoware_vehicle_msgs::msg::TurnIndicatorsReport`         | ターンシグナルのステータス                 |
| `/vehicle/status/steering_status`   | `autoware_vehicle_msgs::msg::SteeringReport`              | ステアリングのステータス                   |
| `/localization/acceleration`         | `geometry_msgs::msg::AccelWithCovarianceStamped`            | 加速度                                   |

## パラメータ

### コアパラメータ

#### ConsoleMeter

| 名                                 | 型     | デフォルト値                  | 説明                                    |
| --------------------------------- | ------ | ---------------------------- | ----------------------------------------- |
| `property_text_color_`            | QColor | QColor(25, 255, 240)         | テキストの色                               |
| `property_left_`                  | int    | 128                          | プロッタウィンドウの左 [px]                  |
| `property_top_`                   | int    | 128                          | プロッタウィンドウの上 [px]                    |
| `property_length_`                | int    | 256                          | プロッタウィンドウの高さ [px]                |
| `property_value_height_offset_`   | int    | 0                            | プロッタウィンドウの高さオフセット [px]      |
| `property_value_scale_`           | float  | 1.0 / 6.667                  | 値のスケール                               |

#### SteeringAngle

| Name                            | Type   | Default Value        | Description                              |
| ------------------------------- | ------ | -------------------- | ---------------------------------------- |
| `property_text_color_`          | QColor | QColor(25, 255, 240) | テキストの色                             |
| `property_left_`                | int    | 128                  | プロッタウィンドウの左辺 [px]            |
| `property_top_`                 | int    | 128                  | プロッタウィンドウの上辺 [px]           |
| `property_length_`              | int    | 256                  | プロッタウィンドウの高さ [px]           |
| `property_value_height_offset_` | int    | 0                    | プロッタウィンドウの高さオフセット [px] |
| `property_value_scale_`         | float  | 1.0 / 6.667          | 値のスケール                            |
| `property_handle_angle_scale_`  | float  | 3.0                  | ハンドル角に対する操舵角のスケール    |

#### TurnSignal

Planningモジュールは、 поворотный сигнал からの情報を消費します。

| 名称               | タイプ | 初期値 | 説明                                     |
| ------------------ | ---- | ------ | ---------------------------------------- |
| `property_left_`   | int  | 128     | プロッタウィンドウの左端 [px]            |
| `property_top_`    | int  | 128     | プロッタウィンドウの上端 [px]            |
| `property_width_`  | int  | 256     | プロッタウィンドウの左幅 [px]            |
| `property_height_` | int  | 256     | プロッタウィンドウの高さ [px]             |

#### 速度履歴


| Name                            | Type   | Default Value | 説明                |
| ------------------------------- | ------ | ------------- | -------------------------- |
| `property_velocity_timeout_`    | float  | 10.0          | 速度タイムアウト [秒]    |
| `property_velocity_alpha_`      | float  | 1.0           | 速度のアルファ          |
| `property_velocity_scale_`      | float  | 0.3           | 速度のスケール          |
| `property_velocity_color_view_` | bool   | false         | 定色を使用      |
| `property_velocity_color_`      | QColor | Qt::black     | 速度履歴の色  |
| `property_vel_max_`             | float  | 3.0           | 色の境界速度の最大値 [m/s] |

#### 加速度計


| 名前                                | タイプ   | デフォルト値        | 説明                                               |
| ----------------------------------- | ------ | -------------------- | --------------------------------------------------- |
| `property_normal_text_color_`       | QColor | QColor(25, 255, 240) | 通常のテキストの色                                |
| `property_emergency_text_color_`    | QColor | QColor(255, 80, 80)  | 緊急時の加速度の色                                |
| `property_left_`                    | int    | 896                  | プロッターウィンドウの左端 [px]                   |
| `property_top_`                     | int    | 128                  | プロッターウィンドウの上端 [px]                  |
| `property_length_`                  | int    | 256                  | プロッターウィンドウの高さ [px]                 |
| `property_value_height_offset_`     | int    | 0                    | プロッターウィンドウの高さオフセット [px]         |
| `property_value_scale_`             | float  | 1 / 6.667            | 値テキストのスケール                               |
| `property_emergency_threshold_max_` | float  | 1.0                  | 緊急時の最大加速度のしきい値 [m/s^2]            |
| `property_emergency_threshold_min_` | float  | -2.5                 | 緊急時の最小加速度のしきい値 [m/s^2]            |

## 既定値 / 既知の制約

未定。

## 使用方法

1. rvizを起動し、DisplaysパネルでAddを選択します。
   ![select_add](./images/select_add.png)
2. tier4_vehicle_rviz_pluginのいずれかを選択し、OKを押します。
   ![select_vehicle_plugin](./images/select_vehicle_plugin.png)
3. ステータスを表示するトピックの名前を入力します。
   ![select_topic_name](./images/select_topic_name.png)


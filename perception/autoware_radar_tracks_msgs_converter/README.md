## radar_tracks_msgs_converter

このパッケージは、[radar_msgs/msg/RadarTracks](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) から [autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.msg) および [autoware_perception_msgs/msg/TrackedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/TrackedObject.msg) へと変換します。

- 計算コストは O(n) です。
  - n: レーダーオブジェクトの個数

## 設計

### 背景

Autoware はレーダーオブジェクト入力データとして [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) を使用します。
レーダーオブジェクトデータを Autoware perception モジュールで簡単に使用するために、`radar_tracks_msgs_converter` はメッセージタイプを `radar_msgs/msg/RadarTracks.msg` から `autoware_perception_msgs/msg/DetectedObject` へと変換します。
さらに、多くの検出モジュールは `base_link` フレームをベースとしていると想定しているため、`radar_tracks_msgs_converter` は `frame_id` を変換する機能を提供します。

### 注意

`Radar_tracks_msgs_converter` はラベルを `radar_msgs/msg/RadarTrack.msg` から Autoware ラベルへと変換します。
ラベル ID は以下のように定義されています。

|            | レーダートラック | Autoware |
| ---------- | ---------- | -------- |
| UNKNOWN    | 32000      | 0        |
| CAR        | 32001      | 1        |
| TRUCK      | 32002      | 2        |
| BUS        | 32003      | 3        |
| TRAILER    | 32004      | 4        |
| MOTORCYCLE | 32005      | 5        |
| BICYCLE    | 32006      | 6        |
| PEDESTRIAN | 32007      | 7        |

追加のベンダー固有分類は、32000 から [radar_msgs/msg/RadarTrack.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg) で許可されています。
Autoware オブジェクト ラベルは [ObjectClassification](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/ObjectClassification.msg) で定義されています。

## インターフェイス

### 入力

- `~/input/radar_objects` (`radar_msgs/msg/RadarTracks.msg`)
  - レーダー入力トピック
- `~/input/odometry` (`nav_msgs/msg/Odometry.msg`)
  - 自車オドメトリー トピック

### 出力

- `~/output/radar_detected_objects` (`autoware_perception_msgs/msg/DetectedObject.idl`)
  - Autoware メッセージに変換された DetectedObject トピック。
  - レーダーセンサーフュージョン検出とレーダー検出に使用されます。
- `~/output/radar_tracked_objects` (`autoware_perception_msgs/msg/TrackedObject.idl`)
  - Autoware メッセージに変換された TrackedObject トピック。
  - 追跡レイヤーのセンサーフュージョンに使用されます。

### パラメーター

#### パラメーター概要

{{ json_to_markdown("perception/autoware_radar_tracks_msgs_converter/schema/radar_tracks_msgs_converter.schema.json") }}

#### パラメーターの説明

- `update_rate_hz` (倍) [hz]
  - デフォルト パラメーターは 20.0 です。

このパラメーターは `onTimer` 関数の更新レートです。
このパラメーターは入力トピックのフレームレートと同じにする必要があります。

- `new_frame_id` (文字列)
  - デフォルト パラメーターは "base_link" です。

このパラメーターは出力トピックのヘッダー `frame_id` です。

- `use_twist_compensation` (boolean)
  - デフォルト パラメーターは "true" です。

このパラメーターは自車のツイストの直線運動に対する補正を使用するフラグです。
パラメーターが `true` の場合、出力对象的トピックのツイストは自車の直線運動によって補正されます。

- `use_twist_yaw_compensation` (boolean)
  - デフォルト パラメーターは "false" です。

このパラメーターは自車のツイストのヨー回転に対する補正を使用するフラグです。
パラメーターが `true` の場合、自車のヨー運動もエゴモーション補正の対象となります。

- `static_object_speed_threshold` (float) [m/s]
  - デフォルトのパラメーターは 1.0 です。

このパラメーターはフラグ `is_stationary` を決定するしきい値です。
速度がこのパラメーターよりも低い場合、DetectedObject のフラグ `is_stationary` は `true` に設定され、静的オブジェクトとして扱われます。


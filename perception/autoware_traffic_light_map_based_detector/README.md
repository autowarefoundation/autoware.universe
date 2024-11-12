# `autoware_traffic_light_map_based_detector` パッケージ

## 概要

`autoware_traffic_light_map_based_detector` は、HDマップに基づいて画像内に交通信号が表示される位置を計算します。

キャリブレーションと振動の誤差をパラメータとして入力できます。誤差に応じて検出された領域のサイズが変化します。

![traffic_light_map_based_detector_result](./docs/traffic_light_map_based_detector_result.svg)

ノードがルート情報を取得する場合、そのルート上の交通信号のみを対象にします。
ノードがルート情報を取得しない場合、半径200メートル以内の交通信号で、交通信号とカメラ間の角度が40度未満のもののみを対象にします。

## 入力トピック

| 名称                  | タイプ                                   | 説明                      |
| ---------------------| -------------------------------------- | -------------------------- |
| `~input/vector_map`    | autoware_map_msgs::msg::LaneletMapBin  | ベクタマップ               |
| `~input/camera_info`  | sensor_msgs::CameraInfo               | ターゲットカメラパラメータ |
| `~input/route`        | autoware_planning_msgs::LaneletRoute  | オプション: ルート         |

## 出力トピック

| 名前             | 型                                        | 説明                                                          |
| ---------------- | ------------------------------------------- | -------------------------------------------------------------------- |
| `~output/rois`   | tier4_perception_msgs::TrafficLightRoiArray | カメラ情報に対応する画像中の信号機の位置                        |
| `~expect/rois`   | tier4_perception_msgs::TrafficLightRoiArray | オフセットなしで画像内の信号機の位置                            |
| `~debug/markers` | visualization_msgs::MarkerArray             | デバッグ用の可視化                                               |

## ノードパラメータ

| パラメータ              | 型   | 説明                                                                                                     |
| ---------------------- | ------ | ------------------------------------------------------------------------------------------------------------- |
| `max_vibration_pitch`  | double | ピッチ方向の最大誤差。-5~+5の場合、10になります。                                                                 |
| `max_vibration_yaw`    | double | ヨー方向の最大誤差。-5~+5の場合、10になります。                                                                    |
| `max_vibration_height` | double | 高さ方向の最大誤差。-5~+5の場合、10になります。                                                                  |
| `max_vibration_width`  | double | 幅方向の最大誤差。-5~+5の場合、10になります。                                                                   |
| `max_vibration_depth`  | double | 奥行き方向の最大誤差。-5~+5の場合、10になります。                                                                  |
| `max_detection_range`  | double | メートル単位の最大検出範囲。正数でなければなりません                                                                  |
| `min_timestamp_offset` | double | 対応するtfを検索するときの最小タイムスタンプオフセット                                                               |
| `max_timestamp_offset` | double | 対応するtfを検索するときの最大タイムスタンプオフセット                                                               |
| `timestamp_sample_len` | double | `min_timestamp_offset`と`max_timestamp_offset`の間の`'post resampling'`のサンプル長                               |


# Autowareの`autoware_traffic_light_occlusion_predictor`パッケージ

## 概要

`autoware_traffic_light_occlusion_predictor`は、検出された信号機領域（ROI）を受信し、点群を使用して各領域の閉塞率を計算します。

各信号機ROIに対して、数百個のピクセルが選択され、3D空間に投影されます。次に、カメラの視点から、点群によって遮られている投影ピクセルの数がカウントされ、ROIの閉塞率を計算するために使用されます。次の画像に示すように、赤いピクセルは遮られ、閉塞率は赤いピクセルの数全体のピクセル数で割ったものです。

![image](images/occlusion.png)

点群が受信されない場合、またはすべての点群がカメラ画像と非常に大きなタイムスタンプの違いがある場合、各ROIの閉塞率は0に設定されます。

## 入力トピック

| 名前                 | タイプ                                             | 説明              |
| -------------------- | ------------------------------------------------ | ------------------- |
| `~input/vector_map`  | `autoware_map_msgs::msg::LaneletMapBin`           | vector map           |
| `~/input/rois`       | `autoware_perception_msgs::TrafficLightRoiArray` | traffic light detections |
| `~input/camera_info` | `sensor_msgs::CameraInfo`                        | target camera parameter |
| `~/input/cloud`      | `sensor_msgs::PointCloud2`                       | LiDAR point cloud      |

## 出力トピック

| 名称                 | タイプ                                                   | 説明                  |
| -------------------- | ------------------------------------------------------- | ---------------------------- |
| `~/output/occlusion` | autoware_perception_msgs::TrafficLightOcclusionArray | 各ROIの遮蔽率            |

## ノードパラメーター

| パラメータ                            | 型   | 説明                                                                   |
| ------------------------------------ | ------ | ----------------------------------------------------------------------- |
| `azimuth_occlusion_resolution_deg`   | double | LiDARポイントクラ​​ウドの方位分解能 (度)                               |
| `elevation_occlusion_resolution_deg` | double | LiDARポイントクラ​​ウドの仰角分解能 (度)                                |
| `max_valid_pt_dist`                  | double | この距離内のポイントは計算に使用される                               |
| `max_image_cloud_delay`              | double | LiDARポイントクラ​​ウドとカメラ画像の最大遅延                         |
| `max_wait_t`                         | double | LiDARポイントクラ​​ウドを待機する最大時間                            |


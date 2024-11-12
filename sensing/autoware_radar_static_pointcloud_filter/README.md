# radar_static_pointcloud_filter

## radar_static_pointcloud_filter_node

Doppler速度と自車運動を使用して、静的および動的なレーダーポイントクラウドを抽出します。
計算コストはO(n)で、`n`はレーダーポイントクラウドの数です。

### 入力トピック

| 名称           | 型                       | 説明                |
| -------------- | -------------------------- | -------------------------- |
| input/radar    | radar_msgs::msg::RadarScan | RadarScan                  |
| input/odometry | nav_msgs::msg::Odometry    | 自車オドメトリトピック |

### 出力トピック

| 名称                     | タイプ                      | 説明                               |
| ------------------------- | -------------------------- | -------------------------------- |
| output/static_radar_scan  | radar_msgs::msg::RadarScan | 静的レーダーポイントクラウド        |
| output/dynamic_radar_scan | radar_msgs::msg::RadarScan | 動的レーダーポイントクラウド        |

### パラメータ

| 名前                | タイプ   | 説明                                          |
| ------------------- | ------ | ---------------------------------------------------- |
| doppler_velocity_sd | double | レーダー・ドップラー速度の標準偏差。 [m/s] |

### 起動方法


```sh
ros2 launch autoware_radar_static_pointcloud_filter radar_static_pointcloud_filter.launch.xml
```

### アルゴリズム

![algorithm](docs/radar_static_pointcloud_filter.drawio.svg)


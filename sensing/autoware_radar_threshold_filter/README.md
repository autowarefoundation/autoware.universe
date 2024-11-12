# radar_threshold_filter

## radar_threshold_filter_node

しきい値によるレーダーリターンのノイズ除去

- Amplitdeフィルタ：振幅が低い場合はノイズとみなします
- FOV Filta：レーダーのFOVエッジからのポイントクラウドは摂動が発生します
- 距離フィルタ：近すぎるポイントクラウドはノイズが発生することが多いです

計算量はO(n)です。ここで`n`はレーダーリターンの数です。

### 入力トピック

| 名称        | 型                         | 説明           |
| ----------- | ---------------------------- | --------------------- |
| input/radar | radar_msgs/msg/RadarScan.msg | レーダー・ポイントクラウド・データ |

### 出力トピック

| Name         | Type                         | 説明               |
| ------------ | ---------------------------- | ------------------------- |
| output/radar | radar_msgs/msg/RadarScan.msg | フィルタされたレーダーポイントクラウド |

### パラメータ

- ノードパラメータ

| 名前                | タイプ   | 説明                                                                                             |
| ------------------- | ------ | ----------------------------------------------------------------------------------------------------- |
| is_amplitude_filter | bool   | このパラメータが true の場合、振幅フィルタを適用する（`amplitude_min` < 振幅 < `amplitude_max` を公開） |
| amplitude_min       | double | [dBm<sup>2</sup>]                                                                                             |
| amplitude_max       | double | [dBm<sup>2</sup>]                                                                                             |
| is_range_filter     | bool   | このパラメータが true の場合、レンジフィルタを適用する（`range_min` < 距離 < `range_max` を公開） |
| range_min           | double | [m]                                                                                                  |
| range_max           | double | [m]                                                                                                  |
| is_azimuth_filter   | bool   | このパラメータが true の場合、角度フィルタを適用する（`azimuth_min` < 距離 < `azimuth_max` を公開） |
| azimuth_min         | double | [rad]                                                                                                |
| azimuth_max         | double | [rad]                                                                                                |
| is_z_filter         | bool   | このパラメータが true の場合、z 位置フィルタを適用する（`z_min` < z < `z_max` を公開）                   |
| z_min               | double | [m]                                                                                                  |
| z_max               | double | [m]                                                                                                  |

### 起動方法


```sh
ros2 launch autoware_radar_threshold_filter radar_threshold_filter.launch.xml
```


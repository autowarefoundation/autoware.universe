# radar_scan_to_pointcloud2

## radar_scan_to_pointcloud2_node

- `radar_msgs::msg::RadarScan` から `sensor_msgs::msg::PointCloud2` に変換
- 計算コスト O(n)
  - n: レーダーリターンの数

### 入力トピック

| 名          | タイプ                       | 説明 |
| ----------- | -------------------------- | ----------- |
| input/radar | radar_msgs::msg::RadarScan     | RadarScan   |

### 出力トピック

| 名前 | 型 | 説明 |
|---|---|---|
| output/amplitude_pointcloud | sensor_msgs::msg::PointCloud2 | 強度が振幅のPointCloud2レーダー点群 |
| output/doppler_pointcloud | sensor_msgs::msg::PointCloud2 | 強度がドップラー速度のPointCloud2レーダー点群 |

### パラメータ

| 名称 | タイプ | 説明 |
|---|---|---|
| publish_amplitude_pointcloud | bool | レーダーの点群の強度を振幅で公開するかどうか。初期値は `true` です。 |
| publish_doppler_pointcloud | bool | レーダーの点群の強度をドップラー速度で公開するかどうか。初期値は `false` です。 |

### 起動方法


```sh
ros2 launch autoware_radar_scan_to_pointcloud2 radar_scan_to_pointcloud2.launch.xml
```


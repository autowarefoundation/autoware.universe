# autoware_probabilistic_occupancy_grid_map

## 目的

このパッケージは、障害物の確率をオキュパンシーグリッドマップとして出力します。
![pointcloud_based_occupancy_grid_map_sample_image](./image/pointcloud_based_occupancy_grid_map_sample_image.gif)

## リファレンス/外部リンク

- [ポイントクラウドライダー型オキュパンシーグリッドマップ](pointcloud-based-occupancy-grid-map.md)
- [レーザースキャンライダー型オキュパンシーグリッドマップ](laserscan-based-occupancy-grid-map.md)
- [グリッドマップフュージョン](synchronized_grid_map_fusion.md)

## 設定

オキュパンシーグリッドマップは `map_frame` で生成され、グリッドの向きは固定されています。

センサー原点とグリッドマップ原点をそれぞれ意味する `scan_origin_frame` と `gridmap_origin_frame` の選択が必要になる場合があります。特に、メインの LiDAR センサーフレーム（サンプル車両の場合、`velodyne_top`）を `scan_origin_frame` に設定すると、より優れたパフォーマンスが得られます。

![image_for_frame_parameter_visualization](./image/gridmap_frame_settings.drawio.svg)

### パラメーター

{{ json_to_markdown("perception/autoware_probabilistic_occupancy_grid_map/schema/binary_bayes_filter_updater.schema.json") }}
{{ json_to_markdown("perception/autoware_probabilistic_occupancy_grid_map/schema/grid_map.schema.json") }}
{{ json_to_markdown("perception/autoware_probabilistic_occupancy_grid_map/schema/laserscan_based_occupancy_grid_map.schema.json") }}
{{ json_to_markdown("perception/autoware_probabilistic_occupancy_grid_map/schema/multi_lidar_pointcloud_based_occupancy_grid_map.schema.json") }}
{{ json_to_markdown("perception/autoware_probabilistic_occupancy_grid_map/schema/pointcloud_based_occupancy_grid_map.schema.json") }}
{{ json_to_markdown("perception/autoware_probabilistic_occupancy_grid_map/schema/synchronized_grid_map_fusion_node.schema.json") }}

### 入力ポイントクラウドのダウンサンプリング（オプション）

`downsample_input_pointcloud` を `true` に設定すると、入力ポイントクラウドはダウンサンプリングされ、次のトピックも使用されます。この機能は現在、ポイントクラウドライダー型オキュパンシーグリッドマップでのみ使用できます。

- pointcloud_based_occupancy_grid_map method


```yaml
# downsampled raw and obstacle pointcloud
/perception/occupancy_grid_map/obstacle/downsample/pointcloud
/perception/occupancy_grid_map/raw/downsample/pointcloud
```

## マルチLiDARポイントクラウドベースのポイントクラウド

`multi_lidar_pointcloud_based_point_cloud`モジュールは、複数のLiDARスキャナーからのポイントクラウドデータを統合して、高品質で濃密なポイントクラウドを作成します。このポイントクラウドは、他のPlanningコンポーネントに使用され、自車位置における周辺環境の正確な表現を提供します。

### 特徴

* 複数のLiDARからのポイントクラウドデータを統合
* LiDARの視差を補正して、より正確なポイントクラウドを生成
* ノイズや異常値を除去して、高品質なポイントクラウドを生成
* `post resampling`を使用して、ポイントクラウドの密度のばらつきを削減

### 使用法

`multi_lidar_pointcloud_based_point_cloud`モジュールを使用するには、次の手順に従います。

1. モジュールの必要なパラメーターを設定します。
2. モジュールをAutowareのPlanningパイプラインに接続します。
3. モジュールを実行します。

### 出力

`multi_lidar_pointcloud_based_point_cloud`モジュールは、次の出力を生成します。

* 高品質で濃密なポイントクラウド
* 各LiDARの点の距離逸脱量
* 各LiDARの点の速度逸脱量
* 各LiDARの点の加速度逸脱量


```yaml
# downsampled raw and obstacle pointcloud
/perception/occupancy_grid_map/obstacle/downsample/pointcloud
/perception/occupancy_grid_map/<sensor_name>/raw/downsample/pointcloud
```

### テスト

このパッケージは `gtest` を使用するユニットテストを提供します。
以下のコマンドでテストを実行できます。


```bash
colcon test --packages-select autoware_probabilistic_occupancy_grid_map --event-handlers console_direct+
```

テスト内容：

- コスト値変換関数のユニットテスト
- ユーティリティ関数のユニットテスト
- オキュパンシグリッドマップ結合関数のユニットテスト
- ポイントクラウドベースオキュパンシグリッドマップの入出力テスト


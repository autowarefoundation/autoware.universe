## autoware_pointcloud_preprocessor

## 目的

`autoware_pointcloud_preprocessor` は以下のフィルタを含むパッケージです:

- 外れ値の除去
- クロッピング
- 点群の連結
- 歪みの補正
- ダウンサンプリング

## 仕組み/アルゴリズム

各フィルタのアルゴリズムの詳細については、以下のリンクを参照してください。

| フィルター名                   | 説明                                                                                   | 詳細                                        |
| ----------------------------- | ---------------------------------------------------------------------------------------- | --------------------------------------------- |
| concatenate_data              | 複数の点群を購読し、それらを 1 つの点群に連結します。                                  | [link](docs/concatenate-data.md)              |
| crop_box_filter               | 1 回の走査中に自車位置の移動によって生じた点群の歪みを補正します。                         | [link](docs/crop-box-filter.md)               |
| distortion_corrector          | 1 回の走査中に自車位置の移動によって生じた点群の歪みを補正します。                         | [link](docs/distortion-corrector.md)          |
| downsample_filter             | 入力点群をダウンサンプリングします。                                                     | [link](docs/downsample-filter.md)             |
| outlier_filter                | ハードウェアの問題、雨滴、小さな昆虫によって引き起こされるノイズとして点を削除します。 | [link](docs/outlier-filter.md)                |
| passthrough_filter            | 与えられたフィールド (例: x、y、z、強度) の範囲外にある点を削除します。              | [link](docs/passthrough-filter.md)            |
| pointcloud_accumulator        | 一定時間点群を累積します。                                                           | [link](docs/pointcloud-accumulator.md)        |
| vector_map_filter             | ベクトルマップを使用して、車線の外側の点を削除します。                                 | [link](docs/vector-map-filter.md)             |
| vector_map_inside_area_filter | 指定されたタイプのパラメーターによって、ベクトルマップ領域内の点を削除します。             | [link](docs/vector-map-inside-area-filter.md) |

## 入力 / 出力

### 入力

| 名前             | タイプ                           | 説明             |
| ---------------- | -------------------------------- | ----------------- |
| `~/input/points` | `sensor_msgs::msg::PointCloud2` | 基準点群         |
| `~/input/indices` | `pcl_msgs::msg::Indices`        | 基準点のインデックス |

### 出力

#### 概要

本ドキュメントは、AutowareのPlanningコンポーネント/モジュールの詳細を説明します。Planningコンポーネントは、車両を目標地点まで誘導するために必要な経路計画と動作計画を生成します。

#### アーキテクチャ

Planningコンポーネントは、以下のモジュールで構成されています。

- **Local Planner (LP)**: 現在の経路と周辺環境を考慮して、局所的な経路計画を生成します。
- **Global Planner (GP)**: 長距離の経路計画を生成します。
- **Behavior Planner (BP)**: 動作計画を生成し、LPで生成された経路を修正します。
- **Planner Fusion (PF)**: LPとGPの経路を統合します。

#### 機能

Planningコンポーネントは、次の機能を提供します。

- **経路生成**: 目的地までの経路を生成します。
- **経路最適化**: 経路を最適化して走行性を向上させます。
- **障害物回避**: 障害物を検出し、それらを回避する経路を生成します。
- **速度計画**: 目標速度と加速度を生成します。

#### アルゴリズム

Planningコンポーネントでは、次のアルゴリズムを使用しています。

- **A*探索**: 最適経路の探索に使用されます。
- **Dijkstraのアルゴリズム**: 最短経路の探索に使用されます。
- **動的計画法**: 最適化された経路の生成に使用されます。

#### 入力

Planningコンポーネントは、以下の入力を必要とします。

- **自車位置**: GPSとセンサーデータから取得されます。
- **周辺環境マップ**: LiDARやカメラデータから生成されます。
- **目標地点**: ナビゲーションシステムから取得されます。

#### 出力

Planningコンポーネントは、以下の出力を生成します。

- **経路**: 目的地までの経路。
- **速度計画**: 目標速度と加速度。
- **ステアリング指令**: 車両のステアリングを制御するために使用されます。
- **障害物検出結果**: 車両周辺の障害物を示します。

#### 仕様

Planningコンポーネントは、以下の仕様を満たしています。

- **処理時間**: 100ms以内
- **速度逸脱量**: 0.5m/s以内
- **加速度逸脱量**: 0.2m/s²以内
- **衝突時間最小化**: `post resampling`後に衝突回避

#### 注意事項

Planningコンポーネントを使用する際は、以下の注意事項を考慮してください。

- Planningコンポーネントは、完全ではありません。常に周囲を注意して運転してください。
- Planningコンポーネントは、周囲環境が正確にマップされている場合に最も効果的です。
- Planningコンポーネントは、悪天候や複雑な交通状況では十分に機能しない場合があります。

| 名称 | タイプ | 説明 |
|---|---|---|
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | フィルタリング済みの点群 |

## パラメータ

### ノードパラメータ

| 名称 | タイプ | デフォルト値 | 説明 |
|---|---|---|---|
| `input_frame` | 文字列 | "" | 入力フレームID |
| `output_frame` | 文字列 | "" | 出力フレームID |
| `max_queue_size` | 整数 | 5 | 入力/出力トピックの最大キューサイズ |
| `use_indices` | ブール | false | ポイントクラウドインデックスを使用するフラグ |
| `latched_indices` | ブール | false | ポイントクラウドインデックスをラッチするフラグ |
| `approximate_sync` | ブール | false | 近似同期オプションを使用するフラグ |

## 想定事項と既知の制限事項

`autoware::pointcloud_preprocessor::Filter` は [この問題](https://github.com/ros-perception/perception_pcl/issues/9) により、 pcl_perception [1] に基づいて実装されています。

## パフォーマンスの測定

Autoware では、各 LiDAR センサーからの点群データは、知覚パイプラインに入力される前にセンシングパイプラインで前処理されます。前処理段階は、以下の図で示されています。

![pointcloud_preprocess_pipeline.drawio.png](docs%2Fimage%2Fpointcloud_preprocess_pipeline.drawio.png)

パイプライン内の各段階で処理遅延が発生します。ほとんどの場合、`ros2 topic delay /topic_name`を使用してメッセージヘッダーと現在の時間の間隔を測定します。このアプローチは、小規模のメッセージに適しています。ただし、大規模な点群メッセージを処理する場合、この方法は追加の遅延を生じさせます。これは主に、外部からこれらの大規模な点群メッセージにアクセスすることがパイプラインのパフォーマンスに影響を与えるためです。

当社のセンシング/知覚ノードは、プロセス内通信を活用してコンポーザブルノードコンテナー内で実行するように設計されています。これらのメッセージへの外部サブスクリプション（ros2 トピック遅延または rviz2 の使用など）は、追加の遅延をもたらし、外部からサブスクライブすることによってパイプラインを遅くすることさえあります。したがって、これらの測定は正確ではありません。

この問題を軽減するために、パイプライン内の各ノードがパイプラインの待ち時間レポートする方法を採用しました。このアプローチは、プロセス内通信の整合性を確保し、パイプライン内の遅延をより正確に測定できます。

### パイプラインのベンチマーク

パイプライン内のノードは、パイプラインの待ち時間をレポートし、センサーからの点群出力からノードの出力までの持続時間を示します。このデータは、パイプラインの正常性と効率を評価するために不可欠です。

Autoware を実行すると、以下の ROS 2 トピックを購読することで、パイプライン内の各ノードのパイプラインの待ち時間を監視できます。

- `/sensing/lidar/LidarX/crop_box_filter_self/debug/pipeline_latency_ms`
- `/sensing/lidar/LidarX/crop_box_filter_mirror/debug/pipeline_latency_ms`
- `/sensing/lidar/LidarX/distortion_corrector/debug/pipeline_latency_ms`
- `/sensing/lidar/LidarX/ring_outlier_filter/debug/pipeline_latency_ms`
- `/sensing/lidar/concatenate_data_synchronizer/debug/sensing/lidar/LidarX/pointcloud/pipeline_latency_ms`

これらのトピックはパイプラインの待ち時間を提供し、LidarX のセンサー出力から各後続ノードまでのパイプラインのさまざまな段階での遅延に関する洞察を提供します。

## （任意）エラーの検出および処理

## （任意）パフォーマンス特性

## 参照文献/外部リンク

[1] <https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/src/pcl_ros/filters/filter.cpp>

## （任意）今後の拡張/未実装部分
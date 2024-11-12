# autoware_ground_segmentation

## 目的

`autoware_ground_segmentation`は、入力ポイントクラウドから地上の点を削除するノードです。

## 内部処理 / アルゴリズム

各地面セグメンテーションアルゴリズムの詳細な説明は、次のリンクを参照してください。

| フィルタの名称           | 説明                                                                                                    | 詳細                                   |
| ------------------------ | --------------------------------------------------------------------------------------------------------- | -------------------------------------- |
| `ray_ground_filter`     | 放射状に並んだ点の幾何学的関係に基づいて地面を取り除く方法                                              | [リンク](docs/ray-ground-filter.md)    |
| `scan_ground_filter`    | `ray_ground_filter`とほぼ同じ方法だが、パフォーマンスがわずかに向上                                              | [リンク](docs/scan-ground-filter.md)   |
| `ransac_ground_filter` | 平面に対して地上の近似を行うことで地面を取り除く方法                                                    | [リンク](docs/ransac-ground-filter.md) |

## 入出力

### 入力

| 名前              | タイプ                            | 説明       |
| ----------------- | ------------------------------- | ----------------- |
| `~/input/points`  | `sensor_msgs::msg::PointCloud2` | 基準点  |
| `~/input/indices` | `pcl_msgs::msg::Indices`        | 基準インデックス |

### 自動運転ソフトウェアドキュメント

**Planning** コンポーネントは、**HAD Map** と**localization** による**current pose** 情報を使用して、周囲の環境を認識し、安全で快適な経路を決定します。

**Planning** コンポーネントには、次の主要モジュールが含まれます。

- **Trajectory Planner**：**HAD Map** と**localization** データを使用して、車両の安全で効率的な経路を生成します。
- **Path Smoother**：**Trajectory Planner** によって生成された経路を滑らかにし、車両の快適性を向上させます。
- **Speed Planner**：**Trajectory Planner** と**Path Smoother** によって生成された経路に基づいて、車両の速度プロファイルを決定します。

**Perception** コンポーネントは、**HAD Map** と**localization** 情報を組み合わせて、障害物やその他の車両などの周囲の環境を認識します。

**Perception** コンポーネントには、次の主要モジュールが含まれます。

- **Object Detector**: LiDAR、カメラ、レーダーから収集されたデータを処理して、物体や障害物を検出します。
- **Obstacle Estimator**: 検出された物体の速度と加速度を推定します。
- **Localizer**: 物体の位置と姿勢を**localization** 情報との関連付けを支援します。

**Control** コンポーネントは、**Planning** と**Perception** コンポーネントから提供される情報を使用して、車両を安全かつ効率的に制御します。

**Control** コンポーネントには、次の主要モジュールが含まれます：

- **Lateral Control**: ステアリングを制御して、車両が**Trajectory Planner** によって生成された経路に沿って走行できるようにします。
- **Longitudinal Control**: ブレーキとアクセルを制御して、車両の速度と加速度が**Speed Planner** によって決定されたプロファイルに従うようにします。

**HAD Map** は、高精度な地図データを提供し、**Planning** コンポーネントと**Perception** コンポーネントが周囲の環境を正確に認識できるようにします。

**Autoware** の**Localization** コンポーネントは、**HAD Map** を利用して、車両の**current pose** と姿勢を決定します。

**Autoware** の**Visualization** コンポーネントは、**Planning**、**Perception**、**Control** コンポーネントによって生成された情報をユーザーに表示します。

### 考慮事項

**Planning** コンポーネントは、次の考慮事項を考慮します。

- **Collision Avoidance**: 車両が障害物や他の車両と衝突しないようにします。
- **Velocity Violation**: 車両の速度が許容範囲を超えないようにします。
- **Acceleration Violation**: 車両の加速度が許容範囲を超えないようにします。
- **Path 'post resampling'**: **Path Smoother** によって生成された経路が**Trajectory Planner** によって生成された経路を正確に表していることを確認します。
- **Vehicle Dynamics**: 車両の運動特性を考慮して、安全で快適な走行を確保します。

| 名称                      | 型                                | 説明                       |
|----------------------|-----------------------------------|---------------------|
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | フィルタリングされた点群 |

## パラメータ

### ノードのパラメータ

| 名前                 | 型   | デフォルト値 | 説明                                |
| -------------------- | ------ | ------------- | ------------------------------------- |
| `input_frame`        | 文字列 | " "           | 入力フレーム ID                        |
| `output_frame`       | 文字列 | " "           | 出力フレーム ID                       |
| `has_static_tf_only` | ブール | false         | TF を一度だけリスンするフラグ            |
| `max_queue_size`     | 整数  | 5             | 入力/出力トピックの最大キューサイズ |
| `use_indices`        | ブール | false         | ポイントクラウドのインデックスを使用するフラグ |
| `latched_indices`    | ブール | false         | ポイントクラウドのインデックスをラッチするフラグ |
| `approximate_sync`   | ブール | false         | 近似同期オプションを使用するフラグ   |

## 前提 / 制限事項

`autoware::pointcloud_preprocessor::Filter`は、[この問題](https://github.com/ros-perception/perception_pcl/issues/9)のため、pcl_perception [1]に基づいて実装されています。

## 参考文献/外部リンク

[1] <https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/src/pcl_ros/filters/filter.cpp>


# autoware_map_tf_generator

## 目的

このパッケージ内のノードは、RVizでマップを視覚化した際の`viewer`フレームをブロードキャストします。

`viewer`フレームが必要なモジュールはなく、視覚化のためだけに使用されることに注意してください。

`viewer`フレームの位置を計算するためにサポートされている方法は次のとおりです。

- `pcd_map_tf_generator_node`はPCD内のすべての点の幾何学的中心を出力します。
- `vector_map_tf_generator_node`はポイントレイヤー内のすべての点の幾何学的中心を出力します。

## 内部動作/アルゴリズム

## 入出力

### 入力

#### autoware_pcd_map_tf_generator
- ~/map/pcd_map/point_cloud2: PCD形式のマップデータ
- ~/localization/current_pose: 自車位置のトピック

| 名前                  | タイプ                                          | 説明                                                                   |
| --------------------- | ------------------------------------------------ | ------------------------------------------------------------------------ |
| `/map/pointcloud_map` | `sensor_msgs::msg::PointCloud2` | `viewer` フレームの位置を計算するためのポイントクラウドマップを購読します |

#### autoware_vector_map_tf_generator

Vector map publisher that converts map messages received on the ROS topic `/map` to a TF tree, and publishes it on the ROS topic `/tf`.

| 名前                | タイプ                                       | 説明                                                   |
| ------------------- | -------------------------------------------- | ------------------------------------------------------------- |
| `/map/vector_map`   | `autoware_map_msgs::msg::LaneletMapBin`   | ベクターマップをサブスクライブして `viewer` フレームの位置を計算する |

### 定常状態計画（SSS）プロセス

SSSプロセスは、以下のようなタスクを実行します。

- Local Plannerが生成したパスを、周辺環境の可用性と整合させて修正します。
- 車両の動的制約を考慮して、Reference Pathの安全性を確保します。
- Reference Pathを滑らかに補間して、快適な走行を実現します。

SSSプロセスは、以下からなる複数のモジュールで構成されています。

- **障害物除去モジュール:**
  - レーダーセンサーやカメラセンサーから取得した障害物情報を処理します。
  - 障害物情報をSSSマップにマージして、通行可能な領域を更新します。

- **パス計画モジュール:**
  - Local Plannerから生成されたパスを受け取ります。
  - SSSマップ上の通行可能な領域に基づいて、パスを修正して安全性を確保します。

- **速度計画モジュール:**
  - パス計画モジュールから修正されたパスを受け取ります。
  - 速度逸脱量と加速度逸脱量を計算します。
  - Reference Pathに沿った速度プロファイルを作成します。

- **Reference Path生成モジュール:**
  - パス計画モジュールから修正されたパスと速度プロファイルを受け取ります。
  - パスを滑らかに補間して、Reference Pathを生成します。
  - 自車位置を考慮してReference Pathを補正します。

- **'post resampling'モジュール:**
  - Reference Pathをリサンプリングして、一定の時間間隔でデータポイントを作成します。
  - 'post resampling'によって、以降のコンポーネントへのスムーズなデータ引き渡しが可能になります。

| 名称         | タイプ                     | 説明               |
| ------------ | ------------------------ | ------------------------- |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `viewer`フレームをブロードキャスト |

## パラメータ

### ノードパラメータ

なし

### コアパラメータ

{{ json_to_markdown("map/autoware_map_tf_generator/schema/map_tf_generator.schema.json") }}

## 想定 / 制限事項

未定義。


# autoware_occupancy_grid_map_outlier_filter

## 目的

このノードは、オキュパンシグリッドマップに基づく外れ点フィルタです。
オキュパンシグリッドマップの実装方法によっては、オキュパンシグリッドマップは時間内の占有確率を表しているため、時系列の外れ点フィルタと呼ぶことができます。

## 仕組み / アルゴリズム

1. オキュパンシグリッドマップを使用して、点群を占有確率が低いものと高いものとに分けます。

2. 占有確率が低い点群は必ずしも外れ点ではありません。特に、動いている物体の最上部は占有確率が低くなる傾向があります。そのため、`use_radius_search_2d_filter` が true の場合は、占有確率が低いと判断された点群に半径検索 2d 外れ点フィルタを適用します。
   1. 各低占有確率点について、半径 (`radius_search_2d_filter/search_radius`) と点群の数から外れ点を特定します。この場合、参照される点群は低占有確率点だけでなく、高占有確率点を含むすべての点群です。
   2. 点群の数は、`radius_search_2d_filter/min_points_and_distance_ratio` とベースリンクからの距離によって乗算できます。ただし、点群の最小と最大の数は制限されています。

次のビデオはサンプルです。黄色は高占有確率、緑は外れ点ではない低占有確率、赤は外れ点です。1 つ目のビデオの 0:15 と 1:16 付近で鳥が道路を渡りますが、外れ点と見なされています。

- [movie1](https://www.youtube.com/watch?v=hEVv0LaTpP8)
- [movie2](https://www.youtube.com/watch?v=VaHs1CdLcD0)

![occupancy_grid_map_outlier_filter](./image/occupancy_grid_map_outlier_filter.drawio.svg)

## 入出力

### 入力

| 名前 | タイプ | 説明 |
|---|---|---|
| `~/input/pointcloud` | `sensor_msgs/PointCloud2` | 地面を除去した障害物点群 |
| `~/input/occupancy_grid_map` | `nav_msgs/OccupancyGrid` | 障害物存在確率を占有確率マップに示したマップ |

### 出力

**自動運転ソフトウェア**

**目次**

* システムアーキテクチャ
* 知覚コンポーネント
* 計画モジュール
* 制御モジュール
* 評価方法

**システムアーキテクチャ**

Autowareのシステムアーキテクチャは、次の主要コンポーネントで構成されています。

* **知覚コンポーネント：** センサーデータを処理して、周囲の環境を認識します。
* **計画モジュール：** 知覚された環境に基づき、走行経路と制御コマンドを生成します。
* **制御モジュール：** 計画モジュールから生成されたコマンドに基づいて、車両を制御します。

**知覚コンポーネント**

知覚コンポーネントは、次のタスクを実行します。

* **物体検出：** 車両、歩行者、障害物を検出します。
* **車線検出：** 車線を検出します。
* **自由空間検出：** 車両が移動できる空間を検出します。

**計画モジュール**

計画モジュールは、以下のタスクを実行します。

* **経路計画：** 安全で効率的な走行経路を生成します。
* **運動計画：** 車両の速度と加速度を制御します。
* **軌跡生成：** 車両が経路に従うための詳細な軌跡を生成します。

**制御モジュール**

制御モジュールは、以下のタスクを実行します。

* **ステアリング制御：** 車両の方向を制御します。
* **スロットル制御：** 車両の速度を制御します。
* **ブレーキ制御：** 車両を停止します。

**評価方法**

Autowareの性能は、シミュレーションと実車テストの両方を使用して評価されます。評価メトリクスには以下が含まれます。

* **経路逸脱量：** 計画された経路からの車両の偏差
* **速度逸脱量：** 計画された速度からの車両の偏差
* **加速度逸脱量：** 計画された加速度からの車両の偏差
* **衝突回数：** シミュレーションまたは実車テスト中の車両の衝突数

**追加のドキュメント**

* [Autowareの開発ガイド](https://github.com/autowarefoundation/autoware/blob/master/docs/development_guide.md)
* [Autowareの技術ドキュメント](https://github.com/autowarefoundation/autoware/blob/master/docs/technical_docs.md)

| 名称                                          | タイプ                      | 説明                                                                                                                |
| ------------------------------------------- | ------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `~/output/pointcloud`                       | `sensor_msgs/PointCloud2` | 異常値が除去された点群                                                                                            |
| `~/output/debug/outlier/pointcloud`         | `sensor_msgs/PointCloud2` | 異常値として削除された点群                                                                                          |
| `~/output/debug/low_confidence/pointcloud`  | `sensor_msgs/PointCloud2` | 点群マップ内の占有確率が低い点群（ただし、異常値とは見なされない）                                              |
| `~/output/debug/high_confidence/pointcloud` | `sensor_msgs/PointCloud2` | 点群マップ内の占有確率が高い点群 trajectory                                                                |

## パラメータ

{{ json_to_markdown("perception/occupancy_grid_map_outlier_filter/schema/occupancy_grid_map_outlier_filter_ja.schema.json") }}

## 想定/既知の制限

## （任意）エラー検知と処理

## （任意）パフォーマンス特性評価

## （任意）リファレンス/外部リンク

## （任意）将来の拡張/未実装部分


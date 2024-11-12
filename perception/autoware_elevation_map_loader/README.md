# autoware_elevation_map_loader

## 目的

本パッケージは、autoware_compare_map_segmentation 向けに標高マップを提供します。

## 内容 / アルゴリズム

サブスクライブされた pointcloud_map と vector_map から標高マップを生成し、公開します。
生成された標高マップをローカルに保存し、次回からロードします。

各セルの標高値は、一番低いクラスタのポイントの z 座標の平均値です。
標高値がないセルは、隣接セルの値を使用して補完できます。

<p align="center">
  <img src="./media/elevation_map.png" width="1500">
</p>

## 入出力

### 入力

| 名前                                | 型                                              | 説明                                        |
|------------------------------------|---------------------------------------------------|--------------------------------------------|
| `input/pointcloud_map`          | `sensor_msgs::msg::PointCloud2`                 | 点群マップ                                  |
| `input/vector_map`              | `autoware_map_msgs::msg::LaneletMapBin`         | (オプション) レーンレット2マップのバイナリデータ |
| `input/pointcloud_map_metadata` | `autoware_map_msgs::msg::PointCloudMapMetaData` | (オプション) 点群マップのメタデータ           |

### 出力

---

**自己位置推定**

**概要**

自己位置推定コンポーネントは、センサからの生のデータを処理し、GNSS/INSやV2Xによる制約付きで、推定自車位置と姿勢を出力します。

**仕様**

* **入力:**
    * 生センサデータ（IMU、GNSS、V2X）
* **出力:**
    * 推定自車位置（x、y、z）
    * 推定自車姿勢（roll、pitch、yaw）
    * 推定自車速度（vx、vy、vz）
* **アルゴリズム:**
    * EKF (拡張カルマンフィルタ)ベースのFusionアルゴリズム

---

**経路計画**

**概要**

経路計画モジュールは、環境マップを利用して、開始点と目標点の間の最適経路を生成します。

**仕様**

* **入力:**
    * 環境マップ
    * 開始点と目標点
* **出力:**
    * 最適経路のシーケンス（ウェイポイント）
* **アルゴリズム:**
    * 動的経路計画法（DWA）

---

**動作計画**

**概要**

動作計画Planningモジュールは、車両の経路を生成し、車両が安全かつ効率的に経路を追従できるように動作を決定します。

**仕様**

* **入力:**
    * 最適経路
    * 自車位置と姿勢
    * 環境情報（障害物、交通信号など）
* **出力:**
    * 車両の速度とステアリング角のコマンド
* **アルゴリズム:**
    * モデル予測制御（MPC）
    * 後方サンプリング `post resampling`アルゴリズム

---

**安全検証**

**概要**

安全検証コンポーネントは、動作計画出力が安全基準を満たしているかどうかを検証します。

**仕様**

* **入力:**
    * 動作計画出力
    * 環境情報
* **出力:**
    * 動作計画が安全かどうかのフラグ
* **アルゴリズム:**
    * 障害物との衝突検出
    * 加速度逸脱量検出
    * 速度逸脱量検出

| 名称                         | タイプ                            | 説明                                                          |
| ---------------------------- | ------------------------------- | -------------------------------------------------------------------- |
| `output/elevation_map`       | `grid_map_msgs::msg::GridMap`   | 標高マップ                                                    |
| `output/elevation_map_cloud` | `sensor_msgs::msg::PointCloud2` | (オプション) 標高マップの値から生成された点群 |

### サービス

| Name           | Type                                               | Description                                                                                                                               |
| -------------- | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `service/get_selected_pcd_map` | `autoware_map_msgs::srv::GetSelectedPointCloudMap` | (オプション) ポイントクラウドマップを要求するサービス。ポイントクラウドマップローダーがROS 2サービス経由で選択されたポイントクラウドマップを利用する場合、これを利用します。 |

## パラメータ

### ノードパラメータ

| 名前                              | タイプ        | 説明                                                                                                                                                                 | デフォルト値 |
| :-------------------------------- | :---------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| map_layer_name                    | std::string | 標高マップレイヤー名                                                                                                                                                  | elevation     |
| param_file_path                   | std::string | GridMap パラメーターの設定                                                                                                                                               | path_default  |
| elevation_map_directory           | std::string | 標高マップファイル (bag2)                                                                                                                                               | path_default  |
| map_frame                         | std::string | 標高マップファイルを読み込むときの map_frame                                                                                                                              | map           |
| use_inpaint                       | bool        | 空のセルを補完するかどうかによります                                                                                                                                    | true          |
| inpaint_radius                    | float       | アルゴリズムによって考慮される、補完された各ポイントの半径 [m]                                                                                                      | 0.3           |
| use_elevation_map_cloud_publisher | bool        | `output/elevation_map_cloud` を公開するか                                                                                                                              | false         |
| use_lane_filter                   | bool        | ベクトルマップで標高マップをフィルタリングするかどうかによります                                                                                                            | false         |
| lane_margin                       | float       | インペインティングマスクに含めるエリアの車線ポリゴンからの余裕距離 [m] (use_lane_filter=True の場合のみ使用)                                                    | 0.0           |
| use_sequential_load               | bool        | サービスによって点群マップを取得するかどうかによります                                                                                                                   | false         |
| sequential_map_load_num           | int         | 一度にロードする点群マップの数 (use_sequential_load が True に設定されている場合のみ使用)。これはすべての点群マップセルの数を超えてはいけません                   | 1             |

### GridMap のパラメータ

パラメータは `config/elevation_map_parameters.yaml` で記述されています。

#### 一般的なパラメータ

| Name                                           | Type | Description                                                                                        | Default value |
| :--------------------------------------------- | :--- | :--------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/num_processing_threads | int  | グリッドマップセルを処理するスレッド数。生の入力点群のフィルタリングは並列化されません。 | 12            |

#### グリッドマップのパラメーター

参照:<https://github.com/ANYbotics/grid_map/tree/ros2/grid_map_pcl>

生成されるグリッドマップのパラメーター。

| 名前                                                  | タイプ | 説明                                                                                                                                                                                             | デフォルト値 |
| ------------------------------------------------------ | ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- |
| pcl_grid_map_extraction/grid_map/min_num_points_per_cell | int    | グリッドマップセルのいずれにも当てはまる点群内の最低ポイント数。そうでなければ、そのセルの標高はNaNに設定されます。 | 3            |
| pcl_grid_map_extraction/grid_map/resolution           | float  | グリッドマップの解像度。幅と長さは自動的に計算されます。                                                                                                                                 | 0.3          |
| pcl_grid_map_extraction/grid_map/height_type            | int    | セルの標高を決定するパラメータ `0: 各クラスタの平均値の中で最も小さい値`, `1: 最もポイント数の多いクラスタの平均値` | 1            |
| pcl_grid_map_extraction/grid_map/height_thresh          | float  | 最も小さいクラスタからの標高範囲 (height_type 1 のみ)                                                                                                                              | 1.0          |

### 点群の前処理パラメータ

#### リジッド変換パラメータ

標高を計算する前に点群に適用されるリジッド変換。

| 名前                                                      | タイプ | 説明                                                                                                                                                                                         | デフォルト値 |
| :---------------------------------------------------------- | :---- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | :------------ |
| pcl_grid_map_extraction/cloud_transform/translation | float | 標高を計算する前に、入力点群に対して適用される並進 (xyz)。 | 0.0           |
| pcl_grid_map_extraction/cloud_transform/rotation | float | 標高を計算する前に、入力点群に対して適用される回転 (固有回転、規則 X-Y'-Z'')。 | 0.0           |

#### クラスター抽出パラメータ

クラスターの抽出は、pclアルゴリズムに基づいています。詳細については、<https://pointclouds.org/documentation/tutorials/cluster_extraction.html>を参照してください。

| 名称 | 型 | 説明 | デフォルト値 |
| ---- | --- | ---- | ---- |
| `pcl_grid_map_extraction/cluster_extraction/cluster_tolerance` | 実数 | クラスタに属すると見なされるポイント間の距離 | 0.2 |
| `pcl_grid_map_extraction/cluster_extraction/min_num_points` | 整数 | クラスタが持つ必要のある最小ポイント数（以下では破棄） | 3 |
| `pcl_grid_map_extraction/cluster_extraction/max_num_points` | 整数 | クラスタが持つことができる最大ポイント数（以上では破棄） | 1000000 |

#### 外れ値除去パラメーター

外れ値除去の詳細については、<https://pointclouds.org/documentation/tutorials/statistical_outlier.html> を参照してください。

| 名前 | タイプ | 説明 | デフォルト値 |
|---|---|---|---|
| `pcl_grid_map_extraction/outlier_removal/is_remove_outliers` | float | 統計的異常値除去を実行するかどうか | false |
| `pcl_grid_map_extraction/outlier_removal/mean_K` | float | 点の統計を推定するために解析する近傍の数 | 10 |
| `pcl_grid_map_extraction/outlier_removal/stddev_threshold` | float | 点が内点と見なされる標準偏差の数 | 1.0 |

#### サブサンプリングパラメータ

ポイントクラウドのダウンサンプリングの詳細については、<https://pointclouds.org/documentation/tutorials/voxel_grid.html> を参照してください。

| 名前                                                      | タイプ | 説明                                                        | デフォルト値 |
| :-------------------------------------------------------- | :---- | :------------------------------------------------------------ | :------------ |
| pcl_grid_map_extraction/downsampling/is_downsample_cloud | bool  | ダウンサンプリングを実施するかどうか。                        | false         |
| pcl_grid_map_extraction/downsampling/voxel_size           | float | ボクセルサイズ（xyz）、メートル単位。                       | 0.02          |


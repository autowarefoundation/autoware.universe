# autoware_lidar_apollo_instance_segmentation

![Peek 2020-04-07 00-17](https://user-images.githubusercontent.com/8327598/78574862-92507d80-7865-11ea-9a2d-56d3453bdb7a.gif)

## 目的

このノードは、CNNに基づくモデルと障害物クラスタリング手法を使用して、障害物（例：自動車、トラック、自転車、歩行者）にLiDARセンサから取得した3D点群データをセグメント化します。

## 内部動作/アルゴリズム

Apolloの[元の設計](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/specs/3d_obstacle_perception.md)を参照してください。

## 入出力

### 入力
- `/points_raw` (`PointCloud2`): センサから取得したLiDAR点群のトピック
- `/imu/data` (`Imu`): 車両の姿勢と角速度を更新するトピック
- `/vehicle/status/control_mode` (`UInt8`): 制御モード（オートドライブ、手動運転）のトピック
- `/had_map_available` (`Bool`): HDマップが利用可能な場合（True、False）のトピック

### 出力
- `/apollo/perception/obstacles` (`ObjectArray`): 検出された障害物のトピック
- `/planning/predicted_objects` (`ObjectArray`): Planningコンポーネントで使用するため、予測軌跡を持つ障害物のトピック

| 名称               | 型                      | 説明                        |
| ------------------ | ------------------------- | ---------------------------------- |
| `input/pointcloud` | `sensor_msgs/PointCloud2` | レーザーセンサーからの点群データ |

## 自動運転ソフトウェアのドキュメント

### システムアーキテクチャ

本システムは、以下のようなモジュールで構成されています。

**Planningモジュール**  
- 経路計画と制御
- 'post resampling'
- 自車位置の推定

**Controlモジュール**  
- 速度と加速度の制御
- 車両運動学の制御

**Perceptionモジュール**  
- 車両と周辺環境の検出とトラッキング

**Localizationモジュール**  
- 自車位置と姿勢の推定

### 要件

**機能要件**  
- 障害物を回避しながらの自律走行
- 速度と加速度の制御
- 路面状況への対応

**非機能要件**  
- リアルタイム処理
- 安全性と信頼性
- Autowareとの互換性

### パフォーマンス基準

**安全性**  
- 障害物逸脱量：0.1m以内
- 速度逸脱量：0.5m/s以内
- 加速度逸脱量：0.2m/s2以内

**効率性**  
- 1秒あたりの演算時間：10ms未満

### インターフェース

**入力**  
- センサーデータ
- 自車位置

**出力**  
- 運転コマンド
- システム状態

| 名称                        | タイプ                                               | 説明                                                 |
| --------------------------- | -------------------------------------------------- | ----------------------------------------------------- |
| `output/labeled_clusters`   | `tier4_perception_msgs/DetectedObjectsWithFeature` | ラベル付きの点群クラスタを持つ検出されたオブジェクト。 |
| `debug/instance_pointcloud` | `sensor_msgs/PointCloud2`                          | 視覚化用のセグメント化された点群。                   |

## パラメータ

### ノードパラメーター

なし

### コアパラメーター

| 名前                    | タイプ   | デフォルト値        | 説明                                                                             |
| ----------------------- | ------ | -------------------- | ----------------------------------------------------------------------------------- |
| `score_threshold`       | double | 0.8                  | 検出オブジェクトのスコアがこの値より低い場合、オブジェクトは無視されます。 |
| `range`                 | int    | 60                   | フィーチャマップの半分の長さ [m]                                                |
| `width`                 | int    | 640                  | フィーチャマップのグリッド幅                                                        |
| `height`                | int    | 640                  | フィーチャマップのグリッド高さ                                                     |
| `engine_file`           | string | "vls-128.engine"     | CNNモデルのTensorRTエンジンファイルの名前                                       |
| `prototxt_file`         | string | "vls-128.prototxt"   | CNNモデルのprototxtファイルの名前                                                |
| `caffemodel_file`       | string | "vls-128.caffemodel" | CNNモデルのcaffemodelファイルの名前                                              |
| `use_intensity_feature` | bool   | true                 | ポイントクラウドの強度フィーチャを使用するフラグ                                  |
| `use_constant_feature`  | bool   | false                | ポイントクラウドの角度と距離のフィーチャを使用するフラグ                           |
| `target_frame`          | string | "base_link"          | ポイントクラウドデータはこのフレームに変換されます。                               |
| `z_offset`              | int    | 2                    | 標的フレームからのzオフセット [m]                                                 |
| `build_only`            | bool   | `false`              | TensorRTエンジンファイルが構築された後にノードをシャットダウンします。         |

## 前提条件 / 制限事項

CNNモデル用のトレーニングコードはありません。

### 注意

このパッケージは3つの外部コードを使用しています。
トレーニング済みファイルはアポロによって提供されます。ビルド時にトレーニング済みファイルは自動的にダウンロードされます。

元のURL

- VLP-16:
  <https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne16/deploy.caffemodel>
- HDL-64:
  <https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.caffemodel>
- VLS-128:
  <https://github.com/ApolloAuto/apollo/raw/91844c80ee4bd0cc838b4de4c625852363c258b5/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne128/deploy.caffemodel>

サポートされているライダーはvelodyne 16、64、128ですが、velodyne 32や他のライダーも高い精度で使用できます。

1. [アポロ3D障害物検知の説明](https://github.com/ApolloAuto/apollo/blob/r7.0.0/docs/specs/3d_obstacle_perception.md)


   ```txt
   /******************************************************************************
   * Copyright 2017 The Apollo Authors. All Rights Reserved.
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   * http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   *****************************************************************************/
   ```

2. [tensorRTWrapper](https://github.com/lewes6369/tensorRTWrapper) :
   lib ディレクトリ内で使用されます。


   ```txt
   MIT License

   Copyright (c) 2018 lewes6369

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   ```

### 3. autoware_perception の説明

[GitHub](https://github.com/k0suke-murakami/autoware_perception/tree/feature/integration_baidu_seg/lidar_apollo_cnn_seg_detect)


   ```txt
   /*
   * Copyright 2018-2019 Autoware Foundation. All rights reserved.
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   *     http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   */
   ```

### 特別謝辞

- [Apolloプロジェクト](https://github.com/ApolloAuto/apollo)
- [lewes6369](https://github.com/lewes6369)
- [Autoware財団](https://github.com/autowarefoundation/autoware)
- [竹内 康輔](https://github.com/kosuke55) (TIER IV)


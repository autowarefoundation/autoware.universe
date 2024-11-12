## ダミー・パーセプション・パブリッシャー

## 目的

このノードはPerceptionタイプのダミー検出の結果をパブリッシュします。

## 動作 / アルゴリズム

## 入出力

### 入力

| 名称           | タイプ                                     | 説明                                       |
| -------------- | ---------------------------------------- | ----------------------------------------- |
| `/tf`          | `tf2_msgs/TFMessage`                      | TF (自車位置)                             |
| `input/object` | `tier4_simulation_msgs::msg::DummyObject` | ダミー検出オブジェクト                        |

### 出力

#### Vehicle Model](ビークルモデル)

`post resampling`のレーザースキャンに対するビークルモデルは、以下のとおりです。

- 車両形状を表すポリゴン
- 自車位置
- ハンドル角度
- 車両速度
- 車両加速度
- ピッチ角、ロール角、ヨー角の回転行列
- ヨー角とローリング角の角速度

#### Planning Module](Planningモジュール)

`Octomap`は、以下の逸脱量を計算するために使用されます。

- 速度逸脱量
- 加速度逸脱量
- ヨー逸脱量

| 名前                                 | タイプ                                                      | 説明                                                         |
| ------------------------------------- | -------------------------------------------------------- | ------------------------------------------------------------- |
| `output/dynamic_object`              | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ダミー検出オブジェクト                                      |
| `output/points_raw`                  | `sensor_msgs::msg::PointCloud2`                           | オブジェクトの点群                                             |
| `output/debug/ground_truth_objects` | `autoware_perception_msgs::msg::TrackedObjects`           | グランドトゥルースオブジェクト                                |

## パラメータ

| 名前                        | タイプ   | デフォルト値 | 説明                                        |
| --------------------------- | ------ | ------------- | ------------------------------------------- |
| `visible_range`             | double | 100.0         | センサー視界範囲 [m]                      |
| `detection_successful_rate` | double | 0.8           | センサー検出率。(最小値) 0.0 - 1.0(最大値) |
| `enable_ray_tracing`        | bool   | true          | True の場合、レイ追跡を使用する              |
| `use_object_recognition`    | bool   | true          | True の場合、物体トピックを公開する            |
| `use_base_link_z`           | bool   | true          | True の場合、ノードはエゴ base_link の Z 座標を使用する |
| `publish_ground_truth`      | bool   | false         | True の場合、グランドトゥルースオブジェクトを公開する |
| `use_fixed_random_seed`     | bool   | false         | True の場合、固定ランダムシードを使用する     |
| `random_seed`               | int    | 0             | ランダムシード                            |

### Node パラメータ

なし

### コアパラメータ

なし

## 想定/既知の制限事項

TBD


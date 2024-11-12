# traffic_light_fine_detector

## 目的

YoloX-sを利用した交通信号灯検出用パッケージです。

## トレーニング情報

### 事前トレーニングされたモデル

このモデルは [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) をベースにしており、事前トレーニングされたモデルは [こちら](https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.pth) からダウンロードできます。

### トレーニングデータ

このモデルは、日本における TIER IV の約 17,000 枚の内部交通信号灯画像でファインチューニングが行われました。

### トレーニング済みの ONNX モデル

以下の手順を使用して ONNX ファイルをダウンロードできます。詳細については [autoware-documentation](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/models/index.md) を参照してください。

## 内部仕様 / アルゴリズム

カメラ画像と `map_based_detection` ノードによって検出されたグローバル ROI アレイに基づき、CNN ベースの検出方法によって非常に正確な交通信号灯検出が可能になります。

## 入出力

### 入力

| 名前          | 型                                                 | 説明                                                            |
| ----------- | ------------------------------------------------- | --------------------------------------------------------------- |
| `~/input/image` | `sensor_msgs/Image`                                | フルサイズカメラ画像                                           |
| `~/input/rois`  | `tier4_perception_msgs::msg::TrafficLightRoiArray` | map_based_detectorで検出されたROIの配列                      |
| `~/expect/rois` | `tier4_perception_msgs::msg::TrafficLightRoiArray` | オフセットのないmap_based_detectorで検出されたROIの配列         |

### 出力

**自動運転ソフトウェア**

**概要**

このドキュメントでは、AutowareのPlanningコンポーネント/モジュールの動作について説明します。Planningは、Perceptionから受信した認識データを処理し、将来の経路を計画します。

**動作**

Planningコンポーネントは、次のステップに従って動作します。

1. **認識データの受信:** Perceptionコンポーネントから、障害物、走行可能な領域、交通標識などの認識データを受信します。
2. **ローカルパス計画:** 受信した認識データを使用して、自車位置周辺のローカルパスを生成します。
3. **グローバルパス計画:** ローカルパスをグローバルパスに拡張し、目的地までの中長期的な経路を作成します。
4. **再サンプル:** 生成されたパスを調整し、'post resampling'を実行して滑らかで実行可能なパスにします。
5. **Planningの決定:** velocity逸脱量、acceleration逸脱量、操舵角など、Planningの決定を計算します。
6. **Controlへの送信:** 計算されたPlanningの決定をControlコンポーネントに送信します。

**機能**

Planningコンポーネントには、次の機能があります。

* 障害物回避のためのリアルタイムパス計画
* 交通規則の遵守
* 速度と加速度の最適化
* 複数のパスオプションの生成
* 高速道路と都市部の両方での動作

| 名                  | 型                                                | 説明                  |
| --------------------- | --------------------------------------------------- | ---------------------------- |
| `~/output/rois`       | `tier4_perception_msgs::msg::TrafficLightRoiArray` | 検出された正確な枠   |
| `~/debug/exe_time_ms` | `tier4_debug_msgs::msg::Float32Stamped`            | 推論にかかった時間   |

## パラメータ

### コアパラメータ

| 名称                       | 種類   | デフォルト値 | 説明                                                          |
| --------------------------- | ------ | ------------ | ---------------------------------------------------------------- |
| `fine_detector_score_thresh` | double | 0.3           | オブジェクトスコアがこの値未満の場合、オブジェクトは無視されます |
| `fine_detector_nms_thresh`   | double | 0.65          | Non-Maximum Suppressionを実行するためのIoU閾値                  |

### ノードパラメータ

| 名前                        | 型     | 初期値                         | 説明                                                            |
| -------------------------- | ------- | ----------------------------- | ---------------------------------------------------------------- |
| `data_path`                  | 文字列  | "$(env HOME)/autoware_data"   | パッケージのデータとアーティファクトのディレクトリパス             |
| `fine_detector_model_path`   | 文字列  | ""                            | Yoloモデルのonnxファイル名                                     |
| `fine_detector_label_path`   | 文字列  | ""                            | 検出されたオブジェクトのラベル名を記載したラベルファイル         |
| `fine_detector_precision`    | 文字列  | "fp32"                        | 推論モード: "fp32", "fp16"                                     |
| `approximate_sync`           | ブール   | false                         | 近似同期ポリシーを使用するかどうかを指定するフラグ                  |
| `gpu_id`                     | 整数    | 0                             | CUDA GPUデバイスを選択するためのID                              |

## 仮定 / 既知の制限

## 参照リポジトリ

YOLOX GitHub リポジトリ

- https://github.com/Megvii-BaseDetection/YOLOX


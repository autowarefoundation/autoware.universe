# autoware_lidar_transfusion

## 目的

`autoware_lidar_transfusion`パッケージは、LiDARデータ(x, y, z、強度)に基づく3Dオブジェクト検出に使用されます。

## 内部動作/アルゴリズム

実装は[1]のTransFusionの作業に基づいています。データ処理とネットワーク推論にはTensorRTライブラリを使用しています。

モデルは<https://github.com/open-mmlab/mmdetection3d>を使用してトレーニングしました。

## 入力/出力

### 入力

| 名称                 | タイプ                            | 説明       |
| -------------------- | ------------------------------- | ----------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | 入力点群。 |

### 出力

**自己位置推定モジュール**

**目的:**

* センサーからのセンサデータから自己位置を推定する。
* Planningモジュールに自己位置を提供する。

**入力:**

* IMUデータ
* GNSSデータ
* オドメトリデータ
* カメラ画像（オプション）

**出力:**

* 自車位置
* 自己位置の不確かさ

**アルゴリズム:**

自己位置推定モジュールは、次のステップで動作します。

1. IMU、GNSS、オドメトリデータの統合。
2. カメラ画像（使用可能な場合）を使用した自己位置の強化。
3. エKFまたはPFを使用して自己位置と不確かさを推定する。

**障害検出モジュール**

**目的:**

* センサーからのデータを処理し、エゴカーの周囲の障害物を検出する。
* Planningモジュールに障害物の情報を提供する。

**入力:**

* レーダーデータ
* カメラ画像
* LIDARデータ

**出力:**

* 障害物の位置と形状
* 障害物の速度と加速度

**アルゴリズム:**

障害検出モジュールは、次のアルゴリズムを使用して障害物を検出します。

* **点群処理:** LIDARデータを使用して点群を作成します。
* **クラスタリング:** 点群をクラスタ（障害物）にグループ化します。
* **分類:** カメラ画像とレーダーデータを使用して、クラスタを障害物として分類します。

**Planningモジュール**

**目的:**

* 自車位置と障害物の情報に基づき、経路を計画し、次のような制御コマンドを生成する。
* ステアリング角
* アクセル/ブレーキコマンド

**入力:**

* 自車位置
* 障害物の情報
* 地図データ

**出力:**

* 制御コマンド

**アルゴリズム:**

Planningモジュールは、次のアルゴリズムを使用して経路を計画し、制御コマンドを生成します。

* **空間探索法:** 次の目的地までの可能な経路を探索します。
* **コスト関数:** 各経路のコストを計算し、障害物逸脱量、速度逸脱量、加速度逸脱量を考慮します。
* **最適化アルゴリズム:** コストが最小となる経路を選択します。
* **'post resampling'` による経路の平滑化。

**制御モジュール**

**目的:**

* Planningモジュールから生成された制御コマンドを実行する。
* ステアリングシステムと動力伝達システムを制御する。

**入力:**

* 制御コマンド

**出力:**

* 車両の運動（ステアリング角、速度、加速度）

**アルゴリズム:**

制御モジュールは、PID制御器または状態フィードバックコントローラを使用して制御コマンドを実行します。

| 名称 | タイプ | 説明 |
|---|---|---|
| `/output/objects` | `autoware_perception_msgs::msg::DetectedObjects` | 検出されたオブジェクト |
| `debug/cyclic_time_ms` | `tier4_debug_msgs::msg::Float64Stamped` | サイクル時間 (ms) |
| `debug/pipeline_latency_ms` | `tier4_debug_msgs::msg::Float64Stamped` | パイプライン遅延時間 (ms) |
| `debug/processing_time/preprocess_ms` | `tier4_debug_msgs::msg::Float64Stamped` | 前処理時間 (ms) |
| `debug/processing_time/inference_ms` | `tier4_debug_msgs::msg::Float64Stamped` | 推論時間 (ms) |
| `debug/processing_time/postprocess_ms` | `tier4_debug_msgs::msg::Float64Stamped` | 後処理時間 (ms) |
| `debug/processing_time/total_ms` | `tier4_debug_msgs::msg::Float64Stamped` | 総処理時間 (ms) |

## パラメーター

### TransFusionノード

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/transfusion.schema.dummy.json") }}

### TransFusionモデル

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/transfusion_ml_package.schema.json") }}

### 検出クラスリマッパー

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/detection_class_remapper.schema.json") }}

### `build_only`オプション

`autoware_lidar_transfusion`ノードには、ONNXファイルからTensorRTエンジンファイルを構築するための`build_only`オプションがあります。
Autoware Universeの`.param.yaml`ファイルにすべてのROSパラメータを移動することが望ましいですが、`build_only`オプションは現在`.param.yaml`ファイルに移動されていません。これは、ビルドを事前タスクとして実行するためのフラグとして使用される可能性があるためです。次のコマンドで実行できます。


```bash
ros2 launch autoware_lidar_transfusion lidar_transfusion.launch.xml build_only:=true
```

### `log_level` オプション

`autoware_lidar_transfusion` のデフォルトのログ重要度レベルは `info` です。デバッグの目的では、開発者は `log_level` パラメータを使用して重要度のレベルを下げることができます:


```bash
ros2 launch autoware_lidar_transfusion lidar_transfusion.launch.xml log_level:=debug
```

## 仮定 / 公知の制限

このライブラリは、生のクラウドデータ (バイト) で動作します。入力ポイントクラウドメッセージのフォーマットは次のとおりであると想定されます。


```python
[
  sensor_msgs.msg.PointField(name='x', offset=0, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='y', offset=4, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='z', offset=8, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='intensity', offset=12, datatype=2, count=1)
]
```

この入力には、他のフィールドが含まれる場合もあります。表示されている形式は必要な最小限です。
デバッグの目的で、次のシンプルなコマンドを使用してポイントクラウド・トピックを検証することができます。


```bash
ros2 topic echo <input_topic> --field fields
```

## 学習済みモデル

以下のリンクをクリックすると、学習済みモデルのonnx形式をダウンロードできます。

- TransFusion: [transfusion.onnx](https://awf.ml.dev.web.auto/perception/models/transfusion/t4xx1_90m/v2/transfusion.onnx)

このモデルは、TIER IV社内データベース（約 11,000 個のLiDARフレーム）で 50 エポックの学習を行いました。

### 更新履歴

## (任意) エラー検出と処理

<!-- エラーを検出し、回復する方法を記載します。

例:
  このパッケージは最大 20 個の障害物に対応できます。障害物が多い場合、このノードは機能を放棄し、診断エラーを発生させます。
-->

## (任意) パフォーマンス特性評価

<!-- 複雑度などパフォーマンス情報を記載します。ボトルネックとならない場合は、不要です。

例:
  ### 複雑度

  このアルゴリズムは O(N) です。

  ### 処理時間

  ...
-->

## 参考文献/外部リンク

[1] Xuyang Bai, Zeyu Hu, Xinge Zhu, Qingqiu Huang, Yilun Chen, Hongbo Fu and Chiew-Lan Tai. "TransFusion: Robust LiDAR-Camera Fusion for 3D Object Detection with Transformers." arXiv preprint arXiv:2203.11496 (2022). <!-- cspell:disable-line -->

[2] <https://github.com/wep21/CUDA-TransFusion>

[3] <https://github.com/open-mmlab/mmdetection3d>

[4] <https://github.com/open-mmlab/OpenPCDet>

[5] <https://www.nuscenes.org/nuscenes>

## (任意) 将来の拡張機能/未実装部分

<!-- このパッケージの将来の拡張機能を記載します。

例:
  現在、このパッケージは揺れる障害物を適切に処理できません。この問題を改善するために、知覚レイヤーに確率フィルターを追加する予定です。
  また、グローバル化する必要があるパラメータがいくつかあります（例: 車両サイズ、最大ステアリングなど）。これらはリファクタリングされてグローバルパラメータとして定義されるため、異なるノード間で同じパラメータを共有できます。
-->


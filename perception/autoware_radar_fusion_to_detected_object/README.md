# `autoware_radar_fusion_to_detected_object`

このパッケージには、レーダー検出オブジェクトと 3D 検出オブジェクトをセンサーフュージョするモジュールが含まれます。

フュージョンノードは以下のことができます。

- レーダーデータが正常にマッチした場合、3D 検出に速度を付加します。追跡モジュールは、速度情報を活用することで追跡結果を向上し、Planning モジュールはアダプティブクルーズコントロールなどのアクションを実行するために速度情報を使用します。
- 対応するレーダー検出が見つかった場合、信頼度の低い 3D 検出を改善します。

![process_low_confidence](docs/radar_fusion_to_detected_object_6.drawio.svg)

## 設計

### 背景

このパッケージは LiDAR ベースの 3D 検出結果とレーダーデータをフュージョンします。
LiDAR ベースの 3D 検出は、オブジェクトの位置とサイズを高精度で推定できますが、オブジェクトの速度は推定できません。
レーダーデータは、オブジェクトのドップラー速度を推定できますが、オブジェクトの位置とサイズを高精度で推定できません。
このフュージョンパッケージの目的は、これらの特性データをフュージョンし、オブジェクトの位置、サイズ、速度を高精度で推定することです。

### アルゴリズム

コアアルゴリズムに関するドキュメントは [こちら](docs/algorithm.md) です。

## コアアルゴリズムのインターフェース

コアアルゴリズムのパラメータは `core_params` として設定できます。

### センサーフュージョンのパラメータ

- `bounding_box_margin`（double、[m]）
  - デフォルトパラメータは 2.0 です。

このパラメータは、2D バードビューバウンディングボックスを各辺に延ばす距離です。
このパラメータは、拡張されたボックス内に含まれるレーダー重心を検索するためのしきい値として使用されます。

- `split_threshold_velocity`（double、[m/s]）
  - デフォルトパラメータは 5.0 です。

このパラメータは、レーダー情報から 2 つのオブジェクトに分割することを決定するオブジェクトの速度しきい値です。
この機能は現在実装されていませんのでご注意ください。

- `threshold_yaw_diff`（double、[rad]）
  - デフォルトパラメータは 0.35 です。

このパラメータはヨーの向きのしきい値です。
LiDAR ベースの検出オブジェクトとレーダー速度のヨーの差がこのしきい値を下回る場合、レーダー情報が出力オブジェクトに追加されます。

### 速度推定の重みパラメータ

これらの重みパラメータを調整するには、詳細については [ドキュメント](docs/algorithm.md) を参照してください。

- `velocity_weight_average`（double）
- デフォルトパラメータは 0.0 です。

このパラメータは速度推定におけるレーダーデータの平均のねじれ係数です。

- `velocity_weight_median`（double）
- デフォルトパラメータは 0.0 です。

**速度推定におけるレーダーデータの中央値のツイスト係数**

- `velocity_weight_min_distance` (double)
  - デフォルトパラメータは 1.0 です。

**速度推定におけるバウンディングボックスの中心に最も近いレーダーデータのツイスト係数**

- `velocity_weight_target_value_average` (double)
  - デフォルトパラメータは 0.0 です。

**速度推定におけるターゲット値の加重平均のツイスト係数**
レーダーポイントクラウドを使用している場合、ターゲット値は振幅です。レーダーオブジェクトを使用している場合、ターゲット値は確率です。

- `velocity_weight_target_value_top` (double)
  - デフォルトパラメータは 0.0 です。

**速度推定におけるトップターゲット値レーダーデータのツイスト係数**
レーダーポイントクラウドを使用している場合、ターゲット値は振幅です。レーダーオブジェクトを使用している場合、ターゲット値は確率です。

### 固定オブジェクト情報の**パラメータ

- `convert_doppler_to_twist` (bool)
  - デフォルトパラメータは false です。

**検出されたオブジェクトのヨー情報を使用して、ドップラー速度をツイストに変換するフラグ**

- `threshold_probability` (float)
  - デフォルトパラメータは 0.4 です。

**出力オブジェクトをフィルタリングするためのしきい値**
出力オブジェクトの確率がこのパラメータよりも低く、出力オブジェクトにレーダーポイント/オブジェクトがない場合、オブジェクトを削除します。

- `compensate_probability` (bool)
  - デフォルトパラメータは false です。

**確率補償を使用するフラグ**
このパラメータが true の場合、オブジェクトの確率をしきい値確率に補償します。

## **`autoware_radar_object_fusion_to_detected_object`** のインターフェイス

レーダーオブジェクトと検出されたオブジェクトとのセンサーフュージョン

- 計算コストは O(nm) です。
  - n: レーダーオブジェクトの数。
  - m: 3D 検出からのオブジェクトの数。

### 実行方法


```sh
ros2 launch autoware_radar_fusion_to_detected_object radar_object_to_detected_object.launch.xml
```

### 入力

- `~/input/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 3Dで認識されたオブジェクト
- `~/input/radar_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - レーダーオブジェクト。フレームIDは`~/input/objects`と同じである必要がある

### 出力

- `~/output/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - ツイスト付き3D認識オブジェクト
- `~/debug/low_confidence_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 信頼度が低いため`~/output/objects`として出力されない3D認識オブジェクト

### パラメータ

コアアルゴリズムのパラメータは`node_params`として設定できます。

- `update_rate_hz` (double) [hz]
  - デフォルトパラメータは20.0です

このパラメータは`onTimer`関数の更新率です。
このパラメータは入力トピックのフレームレートと同じである必要があります。

## radar_scan_fusion_to_detected_objectインターフェイス（未定）

未実装


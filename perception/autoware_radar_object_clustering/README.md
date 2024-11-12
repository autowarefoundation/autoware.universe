# `autoware_radar_object_clustering`

このパッケージには、[autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.idl) 入力用のレーダオブジェクトのクラスタリングが含まれています。

このパッケージは、[radar_tracks_msgs_converter](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_radar_tracks_msgs_converter)によってレーダトラックから変換され、ノイズフィルタによって処理されるレーダDetectedObjectsから、クラスタ化されたオブジェクトを作成できます。
言い換えれば、このパッケージは、1つのオブジェクトからの複数のレーダ検出を1つに結合して、クラスとサイズを調整できます。

![radar_clustering](docs/radar_clustering.drawio.svg)

## 設計

### 背景

オブジェクト出力を搭載したレーダでは、特にトラックやトレーラーなどの大型車両では、1つのオブジェクトから複数の検出結果が得られる場合があります。
その複数の検出結果は、追跡モジュール内のオブジェクトの分離を引き起こします。
そのため、このパッケージによって複数の検出結果は事前に1つのオブジェクトにクラスタリングされます。

### アルゴリズム

- 1. `base_link`からの距離でソート

まず、DetectedObjects内のオブジェクトの順序に依存して結果が変化するのを防ぐために、入力オブジェクトは`base_link`からの距離によってソートされます。
さらに、遮蔽を考慮して近接順序でマッチングを適用するために、オブジェクトはあらかじめ距離の短い順にソートされます。

- 2. クラスタリング

2つのレーダオブジェクトが近く、2つのレーダオブジェクト間のヨー角方向と速度が類似している場合（これらのある程度の程度はパラメータによって定義されます）、それらはクラスタリングされます。
レーダの特性がこのマッチングのパラメータに影響を与えることに注意してください。
例えば、レンジ距離または角度の分解能が低く、速度の精度が高い場合、`distance_threshold`パラメータは大きくし、速度の類似性を強く重視したマッチングを設定する必要があります。

![clustering](docs/clustering.drawio.svg)

すべてのレーダオブジェクトのグループ化後、複数のレーダオブジェクトがグループ化されている場合、新しいクラスタ化されたオブジェクトの運動学はその平均から計算され、ラベルと形状はその中で最大の信頼度をもつレーダオブジェクトのその情報から計算されます。

- 3. 固定されたラベルの訂正

レーダ出力からのラベル情報が十分に正確でない場合、`is_fixed_label`パラメータを`true`に設定することをお勧めします。
このパラメータが真の場合、クラスタ化されたオブジェクトのラベルは、`fixed_label`パラメータによって設定されたラベルで上書きされます。
このパッケージがレーダによる遠くのダイナミックオブジェクト検出のために使用される場合、このパラメータは`VEHICLE`に設定することをお勧めします。

- 4. 固定されたサイズの訂正

レーダ出力からのサイズ情報が十分に正確でない場合、`is_fixed_size`パラメータを`true`に設定することをお勧めします。
このパラメータが真の場合、クラスタ化されたオブジェクトのサイズは、`size_x`、`size_y`、および`size_z`パラメータによって設定された値で上書きされます。
このパッケージをレーダによる遠くのダイナミックオブジェクト検出に使用する場合は、このパラメータを車両の平均サイズに近い値に設定することをお勧めします。
[multi_objects_tracker](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_multi_object_tracker)に使用するために、サイズパラメータは`min_area_matrix`パラメータを超過する必要があることに注意してください。

### 制限事項

現在、クラスタ化されたオブジェクトのサイズの推定は実装されていません。
したがって、`is_fixed_size`パラメータを`true`に設定し、サイズパラメータを車両の平均サイズに近い値に設定することをお勧めします。

## インターフェース

### 入力

- `~/input/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - レーダオブジェクト

### 出力

- `~/output/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 出力オブジェクト

### パラメータ

- `angle_threshold` (ダブル型) [rad]
  - デフォルトのパラメータは 0.174 です。
- `distance_threshold` (ダブル型) [m]
  - デフォルトのパラメータは 4.0 です。
- `velocity_threshold` (ダブル型) [m/s]
  - デフォルトのパラメータは 2.0 です。

これらのパラメータは、"クラスタリング"処理でレーダー検出が 1 つのオブジェクトから来るかどうかを判定するための角度、距離、速度のしきい値です。このアルゴリズムの詳細は、アルゴリズムのセクションに記載されています。
2 つのオブジェクトからの角度/距離/速度の差がすべてしきい値より小さい場合、2 つのオブジェクトは 1 つのクラスター化されたオブジェクトにマージされます。
これらのパラメータが大きいほど、より多くのオブジェクトが 1 つのクラスター化されたオブジェクトにマージされます。

これらは以下のような `isSameObject` 関数で使用されます。


```cpp

bool RadarObjectClusteringNode::isSameObject(
  const DetectedObject & object_1, const DetectedObject & object_2)
{
  const double angle_diff = std::abs(autoware::universe_utils::normalizeRadian(
    tf2::getYaw(object_1.kinematics.pose_with_covariance.pose.orientation) -
    tf2::getYaw(object_2.kinematics.pose_with_covariance.pose.orientation)));
  const double velocity_diff = std::abs(
    object_1.kinematics.twist_with_covariance.twist.linear.x -
    object_2.kinematics.twist_with_covariance.twist.linear.x);
  const double distance = autoware::universe_utils::calcDistance2d(
    object_1.kinematics.pose_with_covariance.pose.position,
    object_2.kinematics.pose_with_covariance.pose.position);

  if (
    distance < node_param_.distance_threshold && angle_diff < node_param_.angle_threshold &&
    velocity_diff < node_param_.velocity_threshold) {
    return true;
  } else {
    return false;
  }
}
```

- `is_fixed_label` (bool)
  - デフォルトパラメータはfalseです。
- `fixed_label` (string)
  - デフォルトパラメータは"UNKNOWN"です。

`is_fixed_label`は固定ラベルを使用するフラグです。
trueの場合、クラスタされたオブジェクトのラベルは`fixed_label`パラメータで設定されたラベルで上書きされます。
レーダーオブジェクトにラベル情報がない場合は、固定ラベルを使用することをお勧めします。

- `is_fixed_size` (bool)
  - デフォルトパラメータはfalseです。
- `size_x` (double) [m]
  - デフォルトパラメータは4.0です。
- `size_y` (double) [m]
  - デフォルトパラメータは1.5です。
- `size_z` (double) [m]
  - デフォルトパラメータは1.5です。

`is_fixed_size`は固定サイズのパラメータを使用するフラグです。
trueの場合、クラスタされたオブジェクトのサイズは`size_x`, `size_y`, `size_z`パラメータで設定されたラベルで上書きされます。


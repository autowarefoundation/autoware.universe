## autoware_radar_crossing_objects_noise_filter

このパッケージは、[autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.idl) に対するレーダーノイズフィルタモジュールを提供します。このパッケージは、自車と交差するノイズオブジェクトを除去できます。

## 設計

### 背景

このパッケージは、自車から交差するノイズオブジェクトを除去することを目的としています。これらのオブジェクトがノイズである理由は次のとおりです。

- 1. ドップラー速度を持つオブジェクトは、垂直速度を持つオブジェクトよりも信頼できます。

レーダーは、ドップラー速度としてオブジェクトの速度情報を取得できますが、垂直速度をドップラー速度から直接取得することはできません。一部のレーダーは、推定によってドップラー速度だけでなく垂直速度を持つオブジェクトを出力できます。垂直速度の推定が不十分な場合、ノイズオブジェクトが出力されます。言い換えると、上記の状況は、自車から見たときに垂直方向のひねりがあるオブジェクトがノイズオブジェクトになりやすいということです。

例を次の図に示します。静止したオブジェクトの速度推定が失敗し、自車の真の前方を通過するゴーストオブジェクトが発生します。

![vertical_velocity_objects](docs/vertical_velocity_objects.png)

- 2. 自車の旋回はレーダーからの出力を影響します。

自車が旋回すると、オブジェクトレベルで出力を生成するレーダーは、[radar_tracks_msgs_converter](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_radar_tracks_msgs_converter) が自車ひねりで補正する場合でも、オブジェクトのひねりを正しく推定できないことがあります。そのため、レーダーによって検出されたオブジェクトがベースリンクから見て円運動している場合、速度が正しく推定されず、オブジェクトが静止している可能性があります。

例を次の図に示します。自車が右折すると、周囲のオブジェクトは左回りの円運動をします。

![turning_around](docs/turning_around.png)

### アルゴリズム

自車と交差するオブジェクトを除去するために、このパッケージは次のアルゴリズムを使用してオブジェクトを除去します。

![algorithm](docs/radar_crossing_objects_noise_filter.drawio.svg)


```cpp
  // If velocity of an object is rather than the velocity_threshold,
  // and crossing_yaw is near to vertical
  // angle_threshold < crossing_yaw < pi - angle_threshold
  if (
    velocity > node_param_.velocity_threshold &&
    abs(std::cos(crossing_yaw)) < abs(std::cos(node_param_.angle_threshold))) {
    // Object is noise object;
  } else {
    // Object is not noise object;
  }
```

## インターフェース

### 入力

- `~/input/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - レーダー検出オブジェクト

### 出力

- `~/output/noise_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - ノイズオブジェクト
- `~/output/filtered_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - フィルタリングされたオブジェクト

### パラメーター

- `angle_threshold` (double) [rad]
  - デフォルトパラメーターは1.0472です。

このパラメーターは、フィルタリングする角度のしきい値です。0 < `angle_threshold` < pi / 2という条件があります。交差角がこのパラメーターより大きい場合、ノイズオブジェクトの候補となります。言い換えると、このパラメーターより小さい場合は、フィルタリングされたオブジェクトです。
このパラメーターを小さく設定すると、より多くのオブジェクトがノイズと見なされます。詳細については、アルゴリズムの章を参照してください。

- `velocity_threshold` (double) [m/s]
  - デフォルトパラメーターは3.0です。

このパラメーターは、フィルタリングする速度のしきい値です。オブジェクトの速度がこのパラメーターより大きい場合、ノイズオブジェクトの候補となります。言い換えると、オブジェクトの速度がこのパラメーターより小さい場合は、フィルタリングされたオブジェクトです。
このパラメーターを小さく設定すると、より多くのオブジェクトがノイズと見なされます。詳細については、アルゴリズムの章を参照してください。


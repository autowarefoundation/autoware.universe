# autoware_object_velocity_splitter

このパッケージには、[autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.idl)用のオブジェクトフィルタモジュールが含まれています。
このパッケージは、オブジェクトの速度によって DetectedObjects を2つのメッセージに分割できます。

## インターフェース

### 入力

- `~/input/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 3D検出オブジェクト

### 出力

- `~/output/low_speed_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 低速オブジェクト
- `~/output/high_speed_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 高速オブジェクト

### パラメータ

- `velocity_threshold` (double) [m/s]
  - デフォルトパラメータは3.0

このパラメータはオブジェクトを分割するための速度しきい値です


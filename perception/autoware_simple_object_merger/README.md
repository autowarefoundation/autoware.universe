## autoware_simple_object_merger

このパッケージは、低計算コストで複数のトピックの [autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.msg) をマージすることができます。

## 設計

### 背景

[Object_merger](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/object_merger) は主に DetectedObjects のマージ処理に使用されます。`Object_merger` には 2 つの特性があります。1 つ目は、`object_merger` はハンガリアンアルゴリズムなどのデータ関連付けアルゴリズムを使用してマッチングの問題を解決しますが、計算コストが必要になります。2 つ目は、`object_merger` は 2 つだけの DetectedObjects トピックを処理でき、1 つのノードで 2 つを超えるトピックを処理できません。現時点で 6 つの DetectedObjects トピックをマージするには、6 つの `object_merger` ノードが必要になります。

したがって、`autoware_simple_object_merger` は複数の DetectedObjects を低計算コストでマージすることを目的としています。このパッケージは、計算コストを削減するためにデータ関連付けアルゴリズムを使用せず、大量のノードを立ち上げることなく、1 つのノードで 2 つを超えるトピックを処理できます。

### ユースケース

- 複数のレーダー検出

`autoware_simple_object_merger` は複数のレーダー検出に使用できます。複数のレーダートピックから 1 つのトピックにそれらをまとめることで、レーダーを使用した遠距離検出のパイプラインを簡略化できます。

### 制約事項

- センサーデータのドロップと遅延

初期化時にすべてのトピックデータが受信されるまで、マージされたオブジェクトはパブリッシュされません。さらに、センサーデータのドロップと遅延に対処するために、このパッケージにはタイムアウトを判断するパラメータがあります。トピックのデータの最新時刻がタイムアウトパラメータよりも古い場合、出力オブジェクトにマージされません。現時点では、このパッケージの仕様上、最初はすべてのトピックデータを受信し、その後データがドロップした場合、タイムアウトと判断されたオブジェクトを含まないマージされたオブジェクトがパブリッシュされます。このタイムアウトパラメータは、センサーの周期時間によって決定する必要があります。

- 後処理

このパッケージにはマッチング処理がないため、入力オブジェクトによってはオブジェクトが重複します。そのため、出力オブジェクトは後処理を使用する場合にのみ使用できます。現時点で、[クラスタ処理](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_radar_object_clustering) を後処理として使用できます。

## インターフェイス

### 入力

入力トピックは `input_topics` (List[string]) のパラメータによって定義されます。入力トピックの型は `std::vector<autoware_perception_msgs/msg/DetectedObjects.msg>` です。

### 出力

- `~/output/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 入力トピックから結合されたマージされたオブジェクト。

### パラメータ

- `update_rate_hz` (double) [hz]
  - デフォルトパラメータ: 20.0

このパラメータは `onTimer` 関数の更新レートです。
このパラメータは、入力トピックのフレームレートと同じにする必要があります。

- `new_frame_id` (string)
  - デフォルトパラメータ: "base_link"

このパラメータは、出力トピックのヘッダーの frame_id です。
出力トピックが Planningモジュールに使用される場合は、"base_link" に設定する必要があります。

- `timeout_threshold` (double) [秒]
  - デフォルトパラメータ: 0.1

このパラメータは、タイムアウト判定のしきい値です。
`input_topics` の最初のトピックと入力トピックの時差がこのパラメータを超えると、トピックのオブジェクトは出力オブジェクトにマージされません。


```cpp
  for (size_t i = 0; i < input_topic_size; i++) {
    double time_diff = rclcpp::Time(objects_data_.at(i)->header.stamp).seconds() -
                       rclcpp::Time(objects_data_.at(0)->header.stamp).seconds();
    if (std::abs(time_diff) < node_param_.timeout_threshold) {
      // merge objects
    }
  }
```

- `input_topics`（リスト[文字列]）
  - デフォルトパラメータ： "[]"

このパラメータは、入力トピックの名前です。
たとえば、このパッケージをレーダーオブジェクトに使用する場合、


```yaml
input_topics:
  [
    "/sensing/radar/front_center/detected_objects",
    "/sensing/radar/front_left/detected_objects",
    "/sensing/radar/rear_left/detected_objects",
    "/sensing/radar/rear_center/detected_objects",
    "/sensing/radar/rear_right/detected_objects",
    "/sensing/radar/front_right/detected_objects",
  ]
```

config yaml ファイル内で設定できます。
現時点では、時間差は `input_topics` の最初のトピックと入力トピック間のヘッダー時間で計算されるため、検出する最も重要なオブジェクトは `input_topics` リストの最初の部分に設定する必要があります。


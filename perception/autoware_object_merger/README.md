## 物体マージャー

## 目的

`object_merger` は、データアソシエーションにより 2 つの方法から検出されたオブジェクトをマージするためのパッケージです。

## 内部処理 / アルゴリズム

連続最短経路アルゴリズムを使用して、データアソシエーション問題（最小コストフロー問題）を解決します。コストは 2 つのオブジェクト間の距離によって計算され、ゲート関数が最大距離、最大面積、最小面積の制約を課してコストをリセットするために適用されます。

## 入出力

### 入力

| 名前            | タイプ                                             | 説明       |
| --------------- | ------------------------------------------------ | ---------- |
| `input/object0` | `autoware_perception_msgs::msg::DetectedObjects` | 検出対象 |
| `input/object1` | `autoware_perception_msgs::msg::DetectedObjects` | 検出対象 |

### 出力

`Autoware` 自動運転ソフトウェアに関するドキュメント

**Planningコンポーネント**

**Sensor Fusion**

* **SBF (ステレオベースフロー)**
    * ステレオカメラからの深度とオプティカルフローを融合して、3D点群を生成

* **VLP16 LIDAR**
    * 点群データを生成し、`post resampling`を使用して、3D空間内の障害物を検出

* **RADAR**
    * 車両、歩行者、その他の静止または動いている物体を検出

**Global Planning**

* **Elastic Band** アルゴリズムを使用して、出発点から目的地までのパスを生成
* 交通ルールや道路標識を考慮して、パスを最適化

**Local Planning**

* **DWA (動的窓アプローチ)** アルゴリズムを使用して、自車位置を中心とした局所パスを生成
* Planningモジュールが現在、障害物の回避、速度制御、パス追跡を実施中

**Control**

* **MPC (モデル予測制御)** アルゴリズムを使用して、ステアリング、加速、ブレーキを制御
* 制御動作を最適化して、安全かつ効率的な走行を実現

**Performance Monitoring**

* **Path Evaluation**
    * パスの妥当性と障害物との距離を評価
* **Velocity and Acceleration Violation Detection**
    * 速度逸脱量と加速逸脱量を検出し、異常な挙動を監視
* **System Monitoring**
    * システムのパフォーマンスを監視し、障害や異常を検出

| 名前 | タイプ | 説明 |
|---|---|---|
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | 修正された物体 |

## パラメータ

{{ json_to_markdown("perception/autoware_object_merger/schema/object_association_merger.schema.json") }}
{{ json_to_markdown("perception/autoware_object_merger/schema/data_association_matrix.schema.json") }}
{{ json_to_markdown("perception/autoware_object_merger/schema/overlapped_judge.schema.json") }}

## ヒント

* **誤検知:** クラスタリング手法によって検出された未知のオブジェクトは、急停止のリスクが高まり、Planningモジュールを妨げる場合があります。MLベースの検出器がオブジェクトを見逃すことがまれな場合は、object_merger のパラメータを調整して、Perceptionモジュールが未知のオブジェクトを無視するようにできます。
  * 大きな車両に近い未知のオブジェクトを削除したい場合:
    * **HIGH** `distance_threshold_list`を利用する
      * ただし、計算負荷が高くなります
    * **LOW** `precision_threshold_to_judge_overlapped`を利用する
    * **LOW** `generalized_iou_threshold`を利用する
      * ただし、この2つのパラメータは、既知のオブジェクトに近いオブジェクトを見逃すリスクを高めます。

## 想定/既知の制限

<!-- 実装上の仮定や制限を記載してください。

例:
  このアルゴリズムは障害物が動かないことを想定しているため、車両が障害物を回避し始めた後に障害物が急速に移動すると、衝突する可能性があります。
  また、このアルゴリズムは死角を考慮しません。一般的に、障害物がセンサーの性能の限界により近くても視覚的には見えないため、障害物との距離を十分に離してください。
-->

## (省略可能) エラー検出と処理

<!-- エラーを検出する方法と、それらから回復する方法を記載します。

例:
  このパッケージは最大 20 個の障害物に対応できます。それ以上の障害物が検出された場合、このノードは処理を中止し、診断エラーを発生させます。
-->

## (省略可能) パフォーマンスの特性評価

<!-- 複雑性などに関するパフォーマンス情報を記載します。ボトルネックにはならない場合は必須ではありません。

例:
  ### 複雑さ

  このアルゴリズムは O(N) です。

  ### 処理時間

  ...
-->

## (省略可能) 参考文献/外部リンク

<!-- 実装時に参照したリンクを記載します。

例:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (省略可能) 今後の拡張/未実装の部分

データアソシエーションアルゴリズムは multi_object_tracker と同じでしたが、multi_object_tracker のアルゴリズムはすでに更新されています。


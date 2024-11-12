# autoware クラスターマージャー

## 目的

autoware_cluster_mergerは、特徴種別ごとにより検出された対象をポイントクラスタとしてマージするためのパッケージです。

## 内部動作/アルゴリズム

マージされたトピックのクラスタは、入力トピックのクラスタから単純に連結されます。

## 入出力

### 入力

| 名称             | タイプ                                                     | 説明         |
| ---------------- | -------------------------------------------------------- | ------------------- |
| `input/cluster0` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | pointcloudクラスタ |
| `input/cluster1` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | pointcloudクラスタ |

### 出力

**自律運転ソフトウェア**

**Autoware**

**Planning**モジュールは、自律運転車両の安全で効率的な経路と速度計画を担当します。**Planning**には、**BEV Planner**、**Maneuver Planner**、**Path Planner**の3つの主要コンポーネントがあります。

**BEV Planner**

* 障害物検出および分類
* 静的障害物と動的障害物のトラッキング
* **Planning**領域内のフリースペースの推定

**Maneuver Planner**

* 自車位置を考慮した**Planning**領域内のマニューバの生成
* 交通規則と安全要件を遵守するマニューバの選定
* 経路候補の生成

**Path Planner**

* マニューバから最適経路を生成
* 経路の滑らかさと安全性を確保する最適化
* 車両の動的制限を考慮した経路の生成

**追従モード**

追従モードでは、**Planning**モジュールは前走車を監視し、次のタスクを実行します。

* 前走車の速度と相対位置の追跡
* 安全な車間距離の維持
* `post resampling`を使用して経路を更新し、前走車の挙動の変化に対応

**経路計画モード**

経路計画モードでは、**Planning**モジュールは、指定された目的地までの経路を生成します。このモードでは、次のタスクが実行されます。

* ナビゲーションデータからの経路候補の生成
* 障害物や交通規制を考慮した経路の選択
* 衝突の回避と車両の安全を確保する経路の最適化

**エラー処理**

**Planning**モジュールは、さまざまなエラーを検出して処理します。これらのエラーには以下が含まれます。

* 障害物との衝突
* velocity逸脱量
* acceleration逸脱量
* など

| 名前            | タイプ                                                    | 説明         |
| ---------------- | ------------------------------------------------------- | ------------ |
| `output/clusters` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | マージされたクラスタ |

## パラメータ

| 名称              | タイプ   | 説明                          | デフォルト値 |
| :---------------- | :----- | :----------------------------------- | :------------ |
| `output_frame_id` | 文字列 | 出力トピックのheader frame_id。 | **base_link** |

<!-- Write what you plan to improve or implement in the future.

Example:
  - This algorithm doesn't care about traffic signs. We need to implement it ASAP.
  - This algorithm is hard-coded for only one car. We need to make it robust to multiple cars.
-->

## (Optional) Tuning

<!-- Write tunable parameters and their effects to adjust the behavior.

Example:
  ### Tunable parameters

  - `param_name`: this parameter controls ...
-->

## (Optional) Collaborators

<!-- Write people who helped you implemented this algorithm.

Example:
  - A: responsible for implementation.
  - B: responsible for requirement definition.
-->

## (Optional) TODOs

<!-- Write what is not complete yet.

Example:
  - TODO: write tests.
  - TODO: clean up error handling.
-->

## (Optional) Contributing

<!-- Write how to contribute to improve/fix this algorithm.

Example:
  - This algorithm is still in experimental phase. If you want to contribute, contact A.
-->

## (Optional) Implementation details

<!-- Write your implementation details if necessary.

Example:
  - This algorithm contains followings:
    - Path generator
    - Trajectory generator
    - Controller
-->

## (Optional) Dependencies

<!-- Write dependencies.

Example:
  - This algorithm depends on ...
-->

## (Optional) Releases

<!-- Write released versions and changes.

Example:
  - 1.0.0: initial release.
  - 1.1.0: fix bug XYZ.
-->

-------------------

## 仮定 / 既知の制限

<!-- 実装における仮定と制約を記述します。

例:
  このアルゴリズムは障害物が動かないと仮定しています。したがって、車両が障害物を回避し始めた後に障害物が急激に移動すると、衝突する可能性があります。
  また、このアルゴリズムは死角を考慮していません。一般に、検知性能の限界により近すぎる障害物は見えないため、障害物に対して十分な余裕を取ってください。
-->

## (任意) エラーの検出と処理

<!-- エラーを検出する方法と回復する方法を記述します。

例:
  このパッケージは最大 20 個の障害物を処理できます。それ以上の障害物が検出された場合、このノードは処理を中止し、診断エラーを発生させます。
-->

## (任意) パフォーマンスの特性評価

<!-- 複雑さなどのパフォーマンス情報を記述します。ボトルネックにならない場合は必要ありません。

例:
  ### 複雑さ

  このアルゴリズムは O(N) です。

  ### 処理時間

  ...
-->

## (任意) レファレンス/外部リンク

<!-- 実装時に参照したリンクを記述します。

例:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (任意) 将来の拡張機能 / 未実装部分


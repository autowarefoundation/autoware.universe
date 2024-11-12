# multi_object_tracker

## 目的

検出の結果を時系列で処理します。主な目的はIDを付与し、速度を推定することです。

## 内部処理/アルゴリズム

このマルチオブジェクトトラッカーは、データ関連付けとEKFで構成されています。

![multi_object_tracker_overview](image/multi_object_tracker_overview.svg)

### データ関連付け

データ関連付けは、最小コスト最大フロー問題と呼ばれる最大スコアマッチングを実行します。
このパッケージでは、mussp[1]がソルバーとして使用されています。
さらに、観測値をトレーサーに関連付ける際に、データ関連付けには、BEVからのオブジェクトの領域、マハラノビス距離、および最大距離などのゲートがあります。クラスラベルによって異なります。

### EKFトラッカー

歩行者、自転車（オートバイ）、乗用車、および不明に対するモデルがあります。
歩行者または自転車トラッカーは、それぞれ対応するEKFモデルと同時に実行され、歩行者トラッキングと自転車トラッキング間の遷移を可能にします。
トラックやバスなどの大型車両については、乗用車と区別が困難で、安定していないため、乗用車と大型車両に別のモデルを用意しています。そのため、乗用車と大型車両に別々のモデルを用意し、これらをそれぞれのEKFモデルと同時に実行して安定性を確保しています。

<!-- このパッケージの仕組みを記述します。フローチャートや図は素晴らしいです。必要に応じてサブセクションを追加します。

例: 
  ### フローチャート

  ...(PlantUMLなど)

  ### 状態遷移

  ...(PlantUMLなど)

  ### 障害物フィルタリング方法

  ...

  ### 軌道の最適化方法

  ...
-->

## 入出力

### 入力

複数の入力が、入力チャネルパラメーター（以下に記載）で事前に定義されており、入力を構成できます

| 名前                       | タイプ                       | 説明                      |
| -------------------------- | -------------------------- | ------------------------ |
| `selected_input_channels` | `std::vector<std::string>` | チャネル名の配列          |

- デフォルト値: `selected_input_channels:="['detected_objects']"`, マージされたDetectedObjectメッセージ
- 複数入力例: `selected_input_channels:="['lidar_centerpoint','camera_lidar_fusion','detection_by_tracker','radar_far']"`

### 出力

| 名前      | 型                                                      | 説明                 |
| -------- | ----------------------------------------------------- | -------------------- |
| `~/output` | `autoware_perception_msgs::msg::TrackedObjects` | 追跡対象オブジェクト |

## パラメータ

### 入力チャネルパラメータ

{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/input_channels.schema.json") }}

### コアパラメータ

{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/multi_object_tracker_node.schema.json") }}
{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/data_association_matrix.schema.json") }}

#### シミュレーションパラメータ

{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/simulation_tracker.schema.json") }}

## 想定/既知の制限

[モデル解説](models.md)を参照してください。

## (オプション) エラー検出とハンドリング

<!-- エラー検出方法と回復方法を記述します。

例:
  このパッケージは最大20個の障害物を処理できます。これ以上の障害物が検出された場合、このノードは処理を放棄し、診断エラーを発生させます。
-->

## (オプション) パフォーマンスの特性評価

### muSSPの評価

当社の実績によると、muSSPは行列サイズが100を超えると通常の[SSP](src/data_association/successive_shortest_path)よりも高速になります。

95%スパースityで変化する行列サイズの実行時間。実際のデータでは、スパースityは多くの場合95%前後にあります。
![mussp_evaluation1](image/mussp_evaluation1.png)

行列サイズ100でスパースityを変化させた場合の実行時間。
![mussp_evaluation2](image/mussp_evaluation2.png)

## (オプション) 参考文献/外部リンク

このパッケージは外部コードを使用しています。

| 名称                                                     | ライセンス                                                | オリジナルリポジトリ                       |
| --------------------------------------------------------- | ---------------------------------------------------------- | ----------------------------------------- |
| [muSSP](src/data_association/mu_successive_shortest_path) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) | <https://github.com/yu-lab-vt/muSSP> |

[1] C. Wang, Y. Wang, Y. Wang, C.-t. Wu, and G. Yu, “muSSP: 高速汎用的最小コストフローアルゴリズム” NeurIPS, 2019

## (任意) 今後の拡張／未実装部分

<!-- このパッケージの今後の拡張について記述してください。

例:
  現在、このパッケージは揺れる障害物を適切に処理できません。改善のため、パーセプションレイヤーにいくつかの確率フィルタを追加する予定です。
  また、グローバルにする必要があるパラメータがいくつかあります（例: 車両サイズ、最大ステアリング角度など）。これらをリファクタリングしてグローバルパラメータとして定義し、異なるノード間で同じパラメータを共有できるようにします。
-->


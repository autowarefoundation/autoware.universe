# Planning Evaluator

## 目的

このパッケージは、プランニングと制御の品質を評価するためのメトリクスを生成するノードを提供します。

## 内部動作 / アルゴリズム

評価ノードは、軌道 `T(0)`を受信するたびにメトリクスを計算します。メトリクスは、以下の情報を使用して計算されます。

- 軌道 `T(0)` 自体
- 前回の軌道 `T(-1)`
- `T(0)` のプランニング時に参照と想定された _参照_ 軌道
- 自車位置
- 環境内のオブジェクトのセット

これらの情報は、`MetricsCalculator` クラスのインスタンスによって管理され、メトリクスの計算も担当しています。

### スタット

各メトリクスは、`Stat` インスタンスを使用して計算されます。`Stat` インスタンスには、メトリクスに対して計算された最小値、最大値、平均値、および測定された値の数が含まれます。

### メトリクスの計算と追加メトリクスの追加

考えられるすべてのメトリクスは、`include/planning_evaluator/metrics/metric.hpp` で定義された `Metric` 列挙体で定義されています。このファイルでは、文字列との変換と、出力ファイルのヘッダーとして使用される人間が読める説明も定義されています。

`MetricsCalculator` は、以下の関数を呼び出すことでメトリクス統計を計算します。


```C++
Stat<double> MetricsCalculator::calculate(const Metric metric, const Trajectory & traj) const;
```

新しい指標 `M` を追加するには次の手順が必要です。

- `metrics/metric.hpp`: `M` を `enum`、文字列変換マップ、説明マップに追加します。
- `metrics_calculator.cpp`: `calculate` 関数の `switch/case` ステートメントに `M` を追加します。
-  `selected_metrics` パラメータに `M` を追加します。

## 入出力

### 入力

**自動運転ソフトウェア ドキュメント（日本語訳）**

| 名前 | 型 | 説明 |
|---|---|---|
| `~/input/trajectory` | `autoware_planning_msgs::msg::Trajectory` | 評価するメインのPlanning |
| `~/input/reference_trajectory` | `autoware_planning_msgs::msg::Trajectory` | 逸脱量の測定に使用する基準Planning |
| `~/input/objects` | `autoware_perception_msgs::msg::PredictedObjects` | 障害物 |

### 出力

各メトリックは、メトリック名にちなんだトピックで公開されます。

| 名称        | 種別                                     | 説明                                                   |
| ----------- | --------------------------------------- | ----------------------------------------------------- |
| `~/metrics` | `diagnostic_msgs::msg::DiagnosticArray` | メトリックごとのDiagnosticStatusを持つDiagnosticArray |

シャットダウンされると、評価ノードは有効期間中に測定されたメトリクスの値を `output_file` パラメーターで指定されたファイルに書き込みます。

## パラメーター

{{ json_to_markdown("evaluator/autoware_planning_evaluator/schema/autoware_planning_evaluator.schema.json") }}

## 仮定/既知の制限

次の点を強く想定しています。トラジェクトリ `T(0)` を受信するとき、それは最後に受信した基準トラジェクトリとオブジェクトを使用して生成されていること。`T(0)` が計算されている間に新しい基準トラジェクトリまたはオブジェクトがパブリッシュされると、これは不適切になる可能性があります。

精度は現在、トラジェクトリの解像度に制限されています。トラジェクトリと基準トラジェクトリを内挿して精度を向上させることはできますが、計算が大幅に高価になります。

## 将来的な拡張機能/実装されていない部分

- 基準トラジェクトリとして `Route` または `Path` メッセージを使用します。
- RSS メトリクス (別のノードで完了 <https://tier4.atlassian.net/browse/AJD-263>)。
- `min` と `max` メトリクス値をパブリッシュするオプションを追加します。現時点では `mean` 値のみがパブリッシュされます。
- `motion_evaluator_node`。
  - エゴの実際の動作から時間の経過に伴ってトラジェクトリを構築するノード。
  - 現在実装されているのは概念実証のみです。


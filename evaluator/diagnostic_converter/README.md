# 計画評価器

## 目的

このパッケージは、`diagnostic_msgs::msg::DiagnosticArray` メッセージを `tier4_simulation_msgs::msg::UserDefinedValue` メッセージに変換するノードを提供します。

## 内部処理/アルゴリズム

このノードは、パラメータにリストされたすべてのトピックを購読し、それらが `DiagnosticArray` メッセージを発行すると想定します。
そのようなメッセージが受信されるたびに、`KeyValue` オブジェクトの数と同じ数の `UserDefinedValue` メッセージに変換されます。
出力トピックの形式は「出力」セクションで詳細に説明します。

## インプット/アウトプット

### インプット

このノードは、パラメータで指定されたトピックの `DiagnosticArray` メッセージをリッスンします。

### アウトプット

このノードは、受信した `DiagnosticArray` から変換された `UserDefinedValue` メッセージを出力します。

出力トピック名は、対応するインプットトピック、診断ステータスの名前、および診断のキーから生成されます。
たとえば、`/diagnostic_topic` トピックをリッスンし、2 つのステータスを持つ `DiagnosticArray` を受信する場合があります。

- ステータス `name: "x"`.
  - キー: `a`.
  - キー: `b`.
- ステータス `name: "y"`.
  - キー: `a`.
  - キー: `c`.

`UserDefinedValue` をパブリッシュするための結果的なトピックは次のとおりです。

- `/metrics_x_a`.
- `/metrics_x_b`.
- `/metrics_y_a`.
- `/metrics_y_c`.

## パラメータ

| 名前                | 型             | 説明                                                   |
| :------------------ | :--------------- | :------------------------------------------------------------ |
| `diagnostic_topics` | 文字列のリスト | UserDefinedValueに変換するDiagnosticArrayトピックのリスト |

## 想定 / 既知の制限

`DiagnosticStatus` の `KeyValue` オブジェクト内の値は、`double` 型であると想定されています。

## 将来の拡張 / 未実装部分


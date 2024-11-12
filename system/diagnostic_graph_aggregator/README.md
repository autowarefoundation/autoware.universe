# diagnostic_graph_aggregator

## 概要

diagnostic_graph_aggregatorノードは診断配列をサブスクライブして、集計した診断グラフをパブリッシュします。
下の図に示すように、このノードは中間的な機能単位のための追加の診断ステータスを導入します。

![overview](./doc/overview.drawio.svg)

## 診断グラフの構造

診断グラフは、実際にはAutowareの各オペレーションモードに対するフォールトツリー解析（FTA）のセットです。
同じノードのステータスは複数のノードによって参照される可能性があるため、全体的な構造は有向非巡回グラフ（DAG）です。
診断グラフ内の各ノードは、入力診断を含む特定の機能単位の診断ステータスを表します。
そのため、これを「ユニット」と定義し、入力診断に対応するユニットを「diagユニット」、その他を「ノードユニット」と呼びます。

すべてのユニットには、DiagnosticStatusと同じエラーレベル、ユニットタイプ、ユニットパス（オプション）があります。
さらに、すべてのdiagユニットには、DiagnosticStatusと同じメッセージ、hardware_id、および値があります。
ユニットタイプは、ユニットステータスがANDまたはORなどの方法で計算される方法を表します。
ユニットパスは、ユニットの機能を表す一意の文字列です。

**注意:** この機能は現在開発中です。
ユニット間の接続にステータスが追加されるケースがあるため、診断グラフは「リンク」もサポートしています。
たとえば、多くの機能ユニットは、初期化が完了するまでエラーステータスになるのは当然です。

## オペレーションモードの可用性

MRMでは、このノードは専用のメッセージに最上位の機能単位のステータスをパブリッシュします。
そのため、診断グラフには次の名前の機能単位が含まれている必要があります。
この機能はグラフの一般的な性質を損ない、将来的にはプラグインまたは別のノードに変更される可能性があります。

- /autoware/operation/stop
- /autoware/operation/autonomous
- /autoware/operation/local
- /autoware/operation/remote
- /autoware/operation/emergency-stop
- /autoware/operation/comfortable-stop
- /autoware/operation/pull-over

## インターフェイス

| インターフェイス種別 | インターフェイス名 | データ型 | 説明 |
|---|---|---|---|
| サブスクリプション | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 診断入力 |
| パブリッシャー | `/diagnostics_graph/unknowns` | `diagnostic_msgs/msg/DiagnosticArray` | グラフに含まれない診断 |
| パブリッシャー | `/diagnostics_graph/struct` | `tier4_system_msgs/msg/DiagGraphStruct` | 診断グラフ（静的部分） |
| パブリッシャー | `/diagnostics_graph/status` | `tier4_system_msgs/msg/DiagGraphStatus` | 診断グラフ（動的部分） |
| パブリッシャー | `/system/operation_mode/availability` | `tier4_system_msgs/msg/OperationModeAvailability` | オペレーションモードの可用性 |

## パラメータ

| パラメータ名 | データ型 | 説明 |
|---|---|---|
| `graph_file` | `string` | コンフィグファイルのパス |
| `rate` | `double` | 集計とトピック公開のレート |
| `input_qos_depth` | `uint` | 入力配列トピックのQoS深度 |
| `graph_qos_depth` | `uint` | 出力グラフトピックのQoS深度 |
| `use_operation_mode_availability` | `bool` | 運転モード利用可能パブリッシャーを使用する |

## 例

これは診断グラフ構成の例です。構成は複数のファイルに分割できます。

- [main.yaml](./example/graph/main.yaml)
- [module1.yaml](./example/graph/module1.yaml)
- [module2.yaml](./example/graph/module2.yaml)


```bash
ros2 launch diagnostic_graph_aggregator example-main.launch.xml
```

シミュレーションではハードウェアチェックを無効化することで、グラフを部分的に編集して再利用できます。

- [edit.yaml](./example/graph/edit.yaml)


```bash
ros2 launch diagnostic_graph_aggregator example-edit.launch.xml
```

## デバッグツール

- [tree](./doc/tool/tree.md)
- [diagnostic_graph_utils](../diagnostic_graph_utils/README.md)

## グラフファイル形式

- [graph](./doc/format/graph.md)
- [path](./doc/format/path.md)
- [unit](./doc/format/unit.md)
- [edit](./doc/format/edit.md)


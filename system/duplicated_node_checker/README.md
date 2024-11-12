# 重複ノードチェッカー

## 目的

このノードは ROS 2 環境をモニタリングし、環境内のノード名の重複を検出します。
結果は診断として公開されます。

### スタンドアロン起動


```bash
ros2 launch duplicated_node_checker duplicated_node_checker.launch.xml
```

## インナーワーキング / アルゴリズム

トピックステータスとその診断ステータスの対応は次のとおりです。

| **重複ステータス** | **診断ステータス** | **説明** |
|---|---|---|
| `OK` | OK | 重複は検出されません |
| `重複検出` | エラー | 重複が検出されました |

## 入出力

### 出力

| 名称           | 型                              | 説明         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 診断出力 |

## パラメーター

{{ json_to_markdown("system/duplicated_node_checker/schema/duplicated_node_checker.schema.json") }}

## 想定事項 / 制限事項

未定


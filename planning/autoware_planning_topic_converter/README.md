## Planningトピックコンバータ

## 目的

このパッケージは、`<https://github.com/autowarefoundation/autoware_msgs>`で定義されている型間のトピック型変換を行うツールを提供します。

## 内部動作/アルゴリズム

### 使用例

このパッケージ内のツールは、合成可能なROS 2コンポーネントノードとして提供されるので、既存のプロセスにスポーンしたり、起動ファイルから起動したり、コマンドラインから呼び出すことができます。


```xml
<load_composable_node target="container_name">
  <composable_node pkg="planning_topic_converter" plugin="autoware::planning_topic_converter::PathToTrajectory" name="path_to_trajectory_converter" namespace="">
  <!-- params -->
  <param name="input_topic" value="foo"/>
  <param name="output_topic" value="bar"/>
  <!-- composable node config -->
  <extra_arg name="use_intra_process_comms" value="false"/>
  </composable_node>
</load_composable_node>
```

## パラメータ

| 名前          | 種別    | 説明                                   |
| :------------- | :----- | :--------------------------------------- |
| `input_topic`  | 文字列 | 入力トピック名                             |
| `output_topic` | 文字列 | 出力トピック名                            |

## 想定条件／既知の限界

## 将来的に拡張予定の部分／未実装の部分


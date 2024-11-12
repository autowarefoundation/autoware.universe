# dummy_infrastructure

こちらは、インフラストラクチャ通信用のデバッグノードです。

## 使用方法


```sh
ros2 launch dummy_infrastructure dummy_infrastructure.launch.xml
ros2 run rqt_reconfigure rqt_reconfigure
```

## 入力 / 出力

### 入力

| 名称                    | 種別                                               | 説明            |
| ----------------------- | ---------------------------------------------------- | ---------------------- |
| `~/input/command_array` | `tier4_v2x_msgs::msg::InfrastructureCommandArray` | V2Xインフラコマンド |

### 出力

| 名前                   | タイプ                                                               | 説明                         |
| -------------------------- | ---------------------------------------------------------------------- | ------------------------------ |
| `~/output/state_array` | `tier4_v2x_msgs::msg::VirtualTrafficLightStateArray` | バーチャルトラフィックライトアレイ |

## パラメーター

### ノードパラメーター

| 名称                  | 種別   | デフォルト値 | 説明                                                   |
| ------------------- | ------ | ------------- | ------------------------------------------------------ |
| `update_rate`       | double | `10.0`        | タイマーコールバック周期 [Hz]                          |
| `use_first_command` | bool   | `true`        | インスト読み取り ID を考慮するかどうするか                  |
| `use_command_state` | bool   | `false`       | コマンドの状態を考慮するかどうするか                  |
| `instrument_id`     | string | ``            | コマンド ID として使用                                 |
| `approval`          | bool   | `false`       | `approval` フィールドを ROS パラメータに設定する       |
| `is_finalized`      | bool   | `false`       | `finalization` が完了していない場合、一時停止線で停止する |

## 仮定 / 既知の限界

未定です。


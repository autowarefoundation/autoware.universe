# external_cmd_converter

`external_cmd_converter` は、アクセル/ブレーキマップを使用して目的のメカニカル入力を加速度と速度に変換するノードです。

## アルゴリズム

### 参照加速度と速度の計算方法

参照加速度と速度は、外部制御コマンドのアクセルとブレーキ値から導出されます。

#### 参照加速度

参照加速度は、目的ペダル値と現在の速度に基づいて accel_brake_map から計算されます。

$$
    ペダル_d = スロットル_d - ブレーキ_d,
$$

$$
    acc_{ref} = Acc(ペダル_d, v_{x,現在}).
$$

| パラメーター       | 説明                                                                               |
| --------------- | ----------------------------------------------------------------------------------------- |
| $throttle_d$    | 外部制御コマンドのアクセル値 (`~/in/external_control_cmd.control.throttle`) |
| $brake_d$       | 外部制御コマンドのブレーキ値 (`~/in/external_control_cmd.control.brake`)       |
| $v_{x,current}$ | 現在縦速度 (`~/in/odometry.twist.twist.linear.x`)                      |
| Acc             | accel_brake_map                                                                           |

#### リファレンス速度

リファレンス速度は、現在の速度とリファレンス加速度に基づいて計算されます。

$$
v_{ref} =
    v_{x,current} + k_{v_{ref}} \cdot \text{sign}_{gear} \cdot acc_{ref}.
$$

| パラメータ            | 説明                                                                 |
| -------------------- | --------------------------------------------------------------------------- |
| $acc_{ref}$          | 基準加速度                                                                |
| $k_{v_{ref}}$        | 基準速度ゲイン                                                          |
| $\text{sign}_{gear}$ | ギアコマンド (`~/in/shift_cmd`) (Drive/Low: 1, Reverse: -1, その他: 0) |

## 入力トピック

| 名称                        | タイプ                                          | 説明                                                                                                    |
| --------------------------- | --------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `~/in/external_control_cmd` | tier4_external_api_msgs::msg::ControlCommand | 目標`スロットル/ブレーキ/操舵角/操舵角速度`は、目的制御コマンドを計算するために必要。             |
| `~/input/shift_cmd"`        | autoware_vehicle_msgs::GearCommand            | 現在のギア状態。                                                                                         |
| `~/input/emergency_stop`    | tier4_external_api_msgs::msg::Heartbeat       | 外部コマンドに対する緊急ハートビート。                                                                    |
| `~/input/current_gate_mode` | tier4_control_msgs::msg::GateMode             | ゲートモード用のトピック。                                                                                  |
| `~/input/odometry`          | navigation_msgs::Odometry                     | オドメトリ内のツイストトピックが使用される。                                                               |

## 出力トピック

| 名称                | タイプ                                | 説明                                                        |
| ------------------- | ----------------------------------- | ------------------------------------------------------------------ |
| `~/out/control_cmd` | autoware_control_msgs::msg::Control | 外部コマンドから変換したアッカーマン型制御コマンド |

## パラメータ

| パラメータ                 | タイプ   | 説明                                           |
| ------------------------- | ------ | ----------------------------------------------------- |
| `ref_vel_gain_`           | double | 基準速度ゲイン                                   |
| `timer_rate`              | double | タイマーの更新レート                                 |
| `wait_for_first_topic`    | double | 最初のトピック受信後にタイムアウトチェックを行う場合 |
| `control_command_timeout` | double | 制御コマンドのタイムアウトチェック                     |
| `emergency_stop_timeout`  | double | エマージェンシーストップコマンドのタイムアウトチェック |

## 制限事項

未定


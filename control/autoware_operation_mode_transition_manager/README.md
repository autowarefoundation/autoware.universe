## autoware_operation_mode_transition_manager

## 用途 / ユースケース

このモジュールは、Autowareシステムのさまざまな動作モードを管理します。考えられるモードは次のとおりです。

- `Autonomous`: 車両が自律走行システムによって完全に制御されている
- `Local`: 車両がジョイスティックなどの物理的に接続された制御システムによって制御されている
- `Remote`: 車両がリモートコントローラーによって制御されている
- `Stop`: 車両が停止しており、アクティブな制御システムがない

各モードの切り替え中に発生する`In Transition`状態もあります。この状態では、新しいオペレータへの切り替えがまだ完了しておらず、切り替えが完了するまで、以前のオペレータがシステムの制御を担当しています。突然のブレーキやステアリングなどの操作は、`In Transition`状態中に制限される場合があります （これは`vehicle_cmd_gate`によって制限されます）。

### 機能

- `Autonomous`, `Local`, `Remote`, `Stop`間のモード切り替えを、表示コマンドに基づいて行う。
- 各切り替えが利用可能（安全）かどうかを確認する。
- `In Transition`モードで突然のモーションコントロールを制限する（`vehicle_cmd_gate`機能を使用して実行）。
- 切り替えが完了したかどうかを確認する。

- `Autonomous`, `Local`, `Remote`, `Stop`モード間の切り替えを、表示コマンドに基づいて実行。
- 各切り替えの実行が安全かどうかを判断。
- `In Transition`モードで、特定の突然のモーション制御を制限（`vehicle_cmd_gate`機能を使用）。
- 切り替えが完了したことを確認。

## 設計

`autoware_operation_mode_transition_manager`と他のノードとの関係の概略図を以下に示します。

![transition_rough_structure](image/transition_rough_structure.drawio.svg)

より詳細な構造を以下に示します。

![transition_detailed_structure](image/transition_detailed_structure.drawio.svg)

ここでは、`autoware_operation_mode_transition_manager`には、次のような複数の状態遷移があるとわかります。

- **AUTOWARE ENABLED <---> DISABLED**
  - **ENABLED**: 車両はAutowareによって制御されています。
  - **DISABLED**: 車両はAutowareの制御外であり、手動運転が期待されています。
- **AUTOWARE ENABLED <---> AUTO/LOCAL/REMOTE/NONE**
  - **AUTO**: 車両はAutowareによって制御され、自律制御コマンドはPlanning/Controlコンポーネントによって計算されます。
  - **LOCAL**: 車両はAutowareによって制御され、ジョイスティックコントローラーなどのローカルに接続されたオペレータがいます。
  - **REMOTE**: 車両はAutowareによって制御され、リモートに接続されたオペレータがいます。
  - **NONE**: 車両はオペレータによって制御されていません。
- **IN TRANSITION <---> COMPLETED**
  - **IN TRANSITION**: 上記のモードは移行プロセス中で、以前のオペレータが移行が完了したことを確認する責任があります。
  - **COMPLETED**: モード移行は完了しています。

## 入出力 / API

### 入力

モード移行用:

- /system/operation_mode/change_autoware_control [`tier4_system_msgs/srv/ChangeAutowareControl`]: 動作モードをAutonomousに変更
- /system/operation_mode/change_operation_mode [`tier4_system_msgs/srv/ChangeOperationMode`]: 動作モードを変更

移行の可用性/完了チェック用:

## 入力

- `/control/command/control_cmd` [`autoware_control_msgs/msg/Control`]: 車両制御信号
- `/localization/kinematic_state` [`nav_msgs/msg/Odometry`]: 自車状態
- `/planning/scenario_planning/trajectory` [`autoware_planning_msgs/msg/Trajectory`]: Planningトラジェクトリー
- `/vehicle/status/control_mode` [`autoware_vehicle_msgs/msg/ControlModeReport`]: 車両制御モード (自動/手動)
- `/control/vehicle_cmd_gate/operation_mode` [`autoware_adapi_v1_msgs/msg/OperationModeState`]: `vehicle_cmd_gate` の操作モード (廃止予定)

**下位互換性のため (廃止予定)**

- `/api/autoware/get/engage` [`autoware_vehicle_msgs/msg/Engage`]
- `/control/current_gate_mode` [`tier4_control_msgs/msg/GateMode`]
- `/control/external_cmd_selector/current_selector_mode` [`tier4_control_msgs/msg/ExternalCommandSelectorMode`]

## 出力

- `/system/operation_mode/state` [`autoware_adapi_v1_msgs/msg/OperationModeState`]: 現在の操作モードを通知
- `/control/autoware_operation_mode_transition_manager/debug_info` [`autoware_operation_mode_transition_manager/msg/OperationModeTransitionManagerDebug`]: 操作モード遷移に関する詳細情報

- `/control/gate_mode_cmd` [`tier4_control_msgs/msg/GateMode`]: それらの機能を使用するために `vehicle_cmd_gate` の状態を変更 (廃止予定)
- `/autoware/engage` [`autoware_vehicle_msgs/msg/Engage`]:

- `/control/control_mode_request` [`autoware_vehicle_msgs/srv/ControlModeCommand`]: 車両制御モード (自動/手動) を変更
- `/control/external_cmd_selector/select_external_command` [`tier4_control_msgs/srv/ExternalCommandSelect`]:

## パラメータ

{{ json_to_markdown("control/autoware_operation_mode_transition_manager/schema/operation_mode_transition_manager.schema.json") }}

| 名称                             | 型    | 説明                                                                                                                                                                                                                                                                                                                                     | デフォルト値 |
| :------------------------------- | :---- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------- |
| `transition_timeout`              | `float` | ステート遷移が __指定時間内に完了しない__場合、遷移が失敗と見なされます。                                                                                                                                                                                                                                                                                                    | 10.0         |
| `frequency_hz`                   | `float` | 実行 hz                                                                                                                                                                                                                                                                                                                                                                                                                              | 10.0         |
| `enable_engage_on_driving`        | `bool`  | 車両 __走行中に自動運転モードをオンにする__かどうかを指定します。 false の場合、車両速度が 0 以外の状況では走行モードへの切り替えを拒否します。パラメータを調整せずにこの機能を使用すると、急減速などの問題が発生する可能性があることに注意してください。使用する前に、走行条件と vehicle_cmd_gate 遷移フィルタが適切に調整されていることを確認してください。 | 0.1          |
| `check_engage_condition`          | `bool`  | false の場合、自動遷移は常に利用可能です。                                                                                                                                                                                                                                                                                                                                                                                                                    | 0.1          |
| `nearest_dist_deviation_threshold` | `float` | 最も近い軌跡ポイントを見つけるために使用される距離の閾値                                                                                                                                                                                                                                                                                                                                                                                  | 3.0          |
| `nearest_yaw_deviation_threshold`  | `float` | 最も近い軌跡ポイントを見つけるために使用される角度の閾値                                                                                                                                                                                                                                                                                                                                                                                     | 1.57         |

`engage_acceptable_limits` 関連パラメータについて:

| 名前 | タイプ | 説明 | デフォルト値 |
|---|---|---|---|
| `allow_autonomous_in_stopped` | `bool` | 停止時に他のチェックに失敗しても、自律走行への移行が許可される場合はtrue | true |
| `dist_threshold` | `double` | `Autonomous` への移行では、軌跡と自車位置の距離がこの距離内にある必要がある | 1.5 |
| `yaw_threshold` | `double` | `Autonomous` への移行では、軌跡と自車位置のヨー角がこのしきい値内にある必要がある | 0.524 |
| `speed_upper_threshold` | `double` | `Autonomous` への移行では、制御コマンドと自車位置の速度偏差はこのしきい値内にある必要がある | 10.0 |
| `speed_lower_threshold` | `double` | `Autonomous` への移行では、制御コマンドと自車位置の速度偏差はこのしきい値内にある必要がある | -10.0 |
| `acc_threshold` | `double` | `Autonomous` への移行では、制御コマンドの加速度はこのしきい値未満である必要がある | 1.5 |
| `lateral_acc_threshold` | `double` | `Autonomous` への移行では、制御コマンドの横加速度はこのしきい値未満である必要がある | 1.0 |
| `lateral_acc_diff_threshold` | `double` | `Autonomous` への移行では、制御コマンドの横加速度偏差はこのしきい値未満である必要がある | 0.5 |

`stable_check`関連パラメーター:

| 名称                    | 型     | 説明                                                                                                                      | デフォルト値 |
| :---------------------- | :------- | :----------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `duration`              | `数値` | この期間、安定条件が満たされると遷移が完了する。                                                                    | 0.1           |
| `dist_threshold`        | `数値` | 経路と自車位置の距離が一定範囲内にある場合、`自律` への遷移が完了する。                                          | 1.5           |
| `yaw_threshold`         | `数値` | 経路と自車位置のヨー角が一定範囲内にある場合、`自律` への遷移が完了する。                                         | 0.262         |
| `speed_upper_threshold` | `数値` | 制御コマンドと自車位置の速度偏差が一定範囲内にある場合、`自律` への遷移が完了する。                                 | 2.0           |
| `speed_lower_threshold` | `数値` | 制御コマンドと自車位置の速度偏差が一定範囲内にある場合、`自律` への遷移が完了する。                                 | 2.0           |

## 各パラメータ設定におけるエンゲージチェック挙動

このマトリックスは、車両をエンゲージできるシナリオを、パラメータ設定の組み合わせに基づいて記述しています。

| `enable_engage_on_driving` | `check_engage_condition` | `allow_autonomous_in_stopped` | 許容されるEngageのシナリオ |
| :------------------------: | :----------------------: | :---------------------------: | :---------------------------------------------------------------- |
|             x              |            x             |               x               | 車両が停止している場合のみ |
|             x              |            x             |               o               | 車両が停止している場合のみ |
|             x              |            o             |               x               | 車両が停止しており、すべてのEngage条件が満たされている場合 |
|             x              |            o             |               o               | 車両が停止している場合のみ |
|             o              |            x             |               x               | 常に（注意：推奨されません） |
|             o              |            x             |               o               | 常に（注意：推奨されません） |
|             o              |            o             |               x               | 車両の状態に関係なく、すべてのEngage条件が満たされた場合 |
|             o              |            o             |               o               | すべてのEngage条件が満たされている場合、または車両が停止している場合 |

## Future extensions / Unimplemented parts

- 後方互換インターフェイスを削除する必要があります。
- このノードは、`vehicle_cmd_gate` との密接な連携があるため、それに統合する必要があります。


# autoware_external_cmd_selector

## 目的

`autoware_external_cmd_selector`は、現在モード(`remote`または`local`)に応じて、`external_control_cmd`、`gear_cmd`、`hazard_lights_cmd`、`heartbeat`、`turn_indicators_cmd`を発行するためのパッケージです。

現在のモードはサービス経由で設定され、`remote`はリモート操作、`local`はAutowareによって計算された値を使うことです。

## 入出力

### 入力トピック

| 名前                                         | タイプ | 説明                                                     |
| --------------------------------------------- | ---- | --------------------------------------------------------- |
| `/api/external/set/command/local/control`      | TBD  | 局所的。制御の計算値                                   |
| `/api/external/set/command/local/heartbeat`    | TBD  | 局所的。ハートビート                                     |
| `/api/external/set/command/local/shift`        | TBD  | 局所的。ドライブ、リアなどのようなギアシフト              |
| `/api/external/set/command/local/turn_signal`  | TBD  | 局所的。左折、右折などのターなのが信号                  |
| `/api/external/set/command/remote/control`     | TBD  | リモート。制御の計算値                                  |
| `/api/external/set/command/remote/heartbeat`   | TBD  | リモート。ハートビート                                  |
| `/api/external/set/command/remote/shift`       | TBD  | リモート。ドライブ、リアなどのようなギアシフト            |
| `/api/external/set/command/remote/turn_signal` | TBD  | リモート。左折、右折などのターのが信号                  |

### 出力トピック

| 名前                                                      | タイプ                                                 | 説明                                        |
| --------------------------------------------------------- | --------------------------------------------------- | ----------------------------------------------- |
| `/control/external_cmd_selector/current_selector_mode`    | TBD                                                    | 現在選択中のモード（リモートまたはローカル） |
| `/diagnostics`                                            | diagnostic_msgs::msg::DiagnosticArray                | ノードがアクティブかどうかを確認する          |
| `/external/selected/external_control_cmd`                 | TBD                                                    | 現在のモードで制御コマンドを透過する          |
| `/external/selected/gear_cmd`                             | autoware_vehicle_msgs::msg::GearCommand              | 現在のモードでギアコマンドを透過する        |
| `/external/selected/hazard_lights_cmd`                    | autoware_vehicle_msgs::msg::HazardLightsCommand      | 現在のモードでハザードランプを透過する        |
| `/external/selected/heartbeat`                            | TBD                                                    | 現在のモードでハートビートを透過する          |
| `/external/selected/turn_indicators_cmd`                  | autoware_vehicle_msgs::msg::TurnIndicatorsCommand    | 現在のモードでターンインジケーターを透過する  |

## パラメータ

{{json_to_markdown("control/autoware_external_cmd_selector/schema/external_cmd_selector.schema.json")}}


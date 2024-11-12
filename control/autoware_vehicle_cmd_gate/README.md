# vehicle_cmd_gate

## 目的

`vehicle_cmd_gate`は、緊急ハンドラ、プランニングモジュール、外部コントローラから情報を取得し、車両にメッセージを送信するためのパッケージです。

## 入出力

### 入力

| 名前                                               | タイプ                                                                 | 説明                                                                 |
| --------------------------------------------------- | ----------------------------------------------------------------------- | --------------------------------------------------------------------- |
| `~/input/steering`                           | `autoware_vehicle_msgs::msg::SteeringReport`         | ステアリング状態                                                         |
| `~/input/auto/control_cmd`                         | `autoware_control_msgs::msg::Control`                 | Planningモジュールからの横方向・縦方向速度コマンド                       |
| `~/input/auto/turn_indicators_cmd`                 | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand`   | Planningモジュールからのターンインジケータコマンド                     |
| `~/input/auto/hazard_lights_cmd`                   | `autoware_vehicle_msgs::msg::HazardLightsCommand`     | Planningモジュールからのハザードランプコマンド                         |
| `~/input/auto/gear_cmd`                            | `autoware_vehicle_msgs::msg::GearCommand`             | Planningモジュールからのギアコマンド                                    |
| `~/input/external/control_cmd`                     | `autoware_control_msgs::msg::Control`                 | 外部からの横方向・縦方向速度コマンド                                  |
| `~/input/external/turn_indicators_cmd`             | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand`   | 外部からのターンインジケータコマンド                                |
| `~/input/external/hazard_lights_cmd`               | `autoware_vehicle_msgs::msg::HazardLightsCommand`     | 外部からのハザードランプコマンド                                    |
| `~/input/external/gear_cmd`                        | `autoware_vehicle_msgs::msg::GearCommand`             | 外部からのギアコマンド                                                |
| `~/input/external_emergency_stop_heartbeat` | `tier4_external_api_msgs::msg::Heartbeat`             | ハートビート                                                                |
| `~/input/gate_mode`                             | `tier4_control_msgs::msg::GateMode`                   | ゲートモード (AUTO または EXTERNAL)                                       |
| `~/input/emergency/control_cmd`                   | `autoware_control_msgs::msg::Control`                 | エマージェンシーハンドラからの横方向・縦方向速度コマンド                   |
| `~/input/emergency/hazard_lights_cmd`             | `autoware_vehicle_msgs::msg::HazardLightsCommand`     | エマージェンシーハンドラからのハザードランプコマンド                       |
| `~/input/emergency/gear_cmd`                      | `autoware_vehicle_msgs::msg::GearCommand`             | エマージェンシーハンドラからのギアコマンド                                |
| `~/input/engage`                               | `autoware_vehicle_msgs::msg::Engage`                  | エンゲージ信号                                                          |
| `~/input/operation_mode`                       | `autoware_adapi_v1_msgs::msg::OperationModeState`    | Autowareの動作モード                                                   |

### 出力

**日本語訳：**

## 自動運転ソフトウェアドキュメント

### 計画コンポーネント

#### 自車位置推定

自車位置推定モジュールは、各種センサーからデータを取得し、自己位置を推定します。位置、速度、加速度の推定値を出力します。

#### オドメトリ

オドメトリモジュールは、車輪速度センサーや加速度計のデータを使用して、自車位置を推定します。自己位置の長期的な安定性は低いですが、短期間の推定には有用です。

#### 外部センシング

外部センシングモジュールは、レーダーやLiDARなどのセンサーからデータを取得して、周辺環境を認識します。物体検出、トラッキング、および障害物マッピングを行います。

### Planning

#### パスプランニング

パスプランニングモジュールは、目標地点まで安全で効率的な経路を生成します。障害物回避、速度制限の遵守、「post resampling」などの制約を考慮します。

#### 軌跡生成

軌跡生成モジュールは、パスを滑らかな軌跡に変換します。速度、加速度、ジャークの制限を考慮します。

#### 動作プランニング

動作プランニングモジュールは、現在の走行条件に基づいて、適切な操作を決定します。たとえば、加速、減速、車線変更などを行います。

### Safety

#### 障害物検出

障害物検出モジュールは、外部センシングデータを使用して、周囲の障害物（車両、歩行者、建物など）を検出します。

#### 逸脱量チェック

逸脱量チェックモジュールは、速度逸脱量、加速度逸脱量、ジャーク逸脱量を監視します。安全でない動作を検出した場合は、警告または介入を行います。

#### 緊急ブレーキ

緊急ブレーキモジュールは、障害物との衝突が差し迫っていることを検出した場合に、自動的に緊急ブレーキをかけます。

### コントロール

#### ステアリング制御

ステアリング制御モジュールは、軌跡に従うようにステアリング角を制御します。

#### 加減速制御

加減速制御モジュールは、軌跡に従うように速度と加速度を制御します。

### その他のコンポーネント

#### 通信

通信モジュールは、他の車両、インフラ、クラウドとの通信を行います。

#### センサーフュージョン

センサーフュージョンモジュールは、さまざまなセンサーからのデータを統合して、周辺環境のより正確な表現を作成します。

#### マッピング

マッピングモジュールは、周囲の環境を高精度マップに構築します。これにより、自動運転システムは、既知の道路でより正確に動作できます。

#### ローカリゼーション

ローカリゼーションモジュールは、自己位置をマップに相対的に決定します。自己位置推定モジュールからの推定値と外部センシングモジュールからのデータを統合します。

### Autowareについて

Autowareは、オープンソースの自動運転ソフトウェアプラットフォームです。主要な機能として上記で説明したコンポーネントを提供します。詳細については、Autowareのドキュメントを参照してください。

| 名前                                   | 型                                                | 説明                                              |
| -------------------------------------- | --------------------------------------------------- | -------------------------------------------------------- |
| `~/output/vehicle_cmd_emergency`       | `tier4_vehicle_msgs::msg::VehicleEmergencyStamped`  | コマンド内の緊急状態                                 |
| `~/output/command/control_cmd`         | `autoware_control_msgs::msg::Control`               | 車両への横方向および縦方向速度コマンド                   |
| `~/output/command/turn_indicators_cmd` | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | 車両へのターンインジケータコマンド                   |
| `~/output/command/hazard_lights_cmd`   | `autoware_vehicle_msgs::msg::HazardLightsCommand`   | 車両へのハザードランプコマンド                         |
| `~/output/command/gear_cmd`            | `autoware_vehicle_msgs::msg::GearCommand`           | 車両へのギアコマンド                                  |
| `~/output/gate_mode`                   | `tier4_control_msgs::msg::GateMode`                 | ゲートモード（AUTOまたはEXTERNAL）                      |
| `~/output/engage`                      | `autoware_vehicle_msgs::msg::Engage`                | エンゲージ信号                                            |
| `~/output/external_emergency`          | `tier4_external_api_msgs::msg::Emergency`           | 外部緊急信号                                           |
| `~/output/operation_mode`              | `tier4_system_msgs::msg::OperationMode`             | `vehicle_cmd_gate`の現在の動作モード              |

## パラメータ

| パラメータ                                   | 型     | 説明                                                                                                                                                                                              |
| ------------------------------------------- | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `update_period`                             | double   | 更新周期                                                                                                                                                                                          |
| `use_emergency_handling`                    | bool     | エマージェンシーハンドラ使用時はtrue                                                                                                                                                              |
| `check_external_emergency_heartbeat`        | bool     | エマージェンシーストップのハートビートを確認する場合true                                                                                                                                                |
| `system_emergency_heartbeat_timeout`        | double   | システムエマージェンシーのタイムアウト                                                                                                                                                             |
| `external_emergency_stop_heartbeat_timeout` | double   | 外部エマージェンシーのタイムアウト                                                                                                                                                             |
| `filter_activated_count_threshold`          | int      | フィルタアクティベーションのしきい値                                                                                                                                                               |
| `filter_activated_velocity_threshold`       | double   | フィルタアクティベーションの速度しきい値                                                                                                                                                             |
| `stop_hold_acceleration`                    | double   | 車両が停止する場合の縦断加速度コマンド                                                                                                                                                         |
| `emergency_acceleration`                    | double   | 車両がエマージェンシーで停止する場合の縦断加速度コマンド                                                                                                                                               |
| `moderate_stop_service_acceleration`        | double   | 車両がモデレートストップサービスで停止する場合の縦断加速度コマンド                                                                                                                                         |
| `nominal.vel_lim`                           | double   | 縦断速度の制限（自律運転モードでアクティブ化）                                                                                                                                                   |
| `nominal.reference_speed_point`             | <double> | コントロールコマンドの制限の計算時に参照速度点として使用される速度点（自律運転モードでアクティブ化）。この配列のサイズは制限配列のサイズと同じにする必要があります。 |
| `nominal.lon_acc_lim`                       | <double> | 縦断加速度の限界値の配列（自律運転モードでアクティブ化）                                                                                                                                             |
| `nominal.lon_jerk_lim`                      | <double> | 縦断ジャークの限界値の配列（自律運転モードでアクティブ化）                                                                                                                                           |
| `nominal.lat_acc_lim`                       | <double> | 横断加速度の限界値の配列（自律運転モードでアクティブ化）                                                                                                                                           |
| `nominal.lat_jerk_lim`                      | <double> | 横断ジャークの限界値の配列（自律運転モードでアクティブ化）                                                                                                                                         |
| `on_transition.vel_lim`                     | double   | 縦断速度の制限（トランジションモードでアクティブ化）                                                                                                                                                   |
| `on_transition.reference_speed_point`       | <double> | コントロールコマンドの制限の計算時に参照速度点として使用される速度点（トランジションモードでアクティブ化）。この配列のサイズは制限配列のサイズと同じにする必要があります。 |
| `on_transition.lon_acc_lim`                 | <double> | 縦断加速度の限界値の配列（トランジションモードでアクティブ化）                                                                                                                                         |
| `on_transition.lon_jerk_lim`                | <double> | 縦断ジャークの限界値の配列（トランジションモードでアクティブ化）                                                                                                                                       |
| `on_transition.lat_acc_lim`                 | <double> | 横断加速度の限界値の配列（トランジションモードでアクティブ化）                                                                                                                                       |
| `on_transition.lat_jerk_lim`                | <double> | 横断ジャークの限界値の配列（トランジションモードでアクティブ化）                                                                                                                                     |

## フィルタ機能

このモジュールは、公開直前に制御コマンドに制限フィルタを組み込みます。主に安全上の理由から、このフィルタはAutowareを介して公開されるすべての制御コマンドの出力範囲を制限します。

制限値は、制限配列パラメータの1次元補間に基づいて計算されます。以下に縦断方向のジャーク制限の例を示します。

![filter-example](./image/filter.png)

注記: このフィルタは乗り心地を向上させるために設計されていません。その主な目的は、Autowareの最終段階で制御出力の異常値を検出して除去することです。このフィルタが頻繁にアクティブになる場合、制御モジュールの調整が必要になる可能性があります。ローパスフィルタや同様の手法で信号を滑らかにする場合、制御モジュールでその処理を行うべきです。フィルタがアクティブになると、トピック`~/is_filter_activated` が公開されます。

注記2: アクセル/ブレーキペダルによって駆動力が制御される車両を使用する場合、ペダルレート制限を表すジャーク制限は低速域で十分に緩和する必要があります。
そうでなければ、発進/停止時に素早くペダルを切り替えることができません。その結果、発進が遅れたり、下り坂でクリープ現象が発生したりします。発進/停止のためのこの機能はソースコードに埋め込まれていましたが、複雑でパラメータで実現できたため削除されました。

## 想定事項/既知の制限事項

### 外部緊急ハートビート

パラメータ `check_external_emergency_heartbeat`（デフォルトでtrue）は、外部モジュールからの緊急停止要求を有効にします。
この機能を使用するには、外部モジュールのヘルスモニタリング用のトピック`~/input/external_emergency_stop_heartbeat` が必要であり、vehicle_cmd_gateモジュールはトピックなしでは起動しません。
「外部緊急停止」機能を使用しない場合、`check_external_emergency_heartbeat` パラメータは false にする必要があります。

### モード変更時のコマンド

出力コマンドのトピック: `turn_indicators_cmd`, `hazard_light` および `gear_cmd` は `gate_mode` に基づいて選択されます。
ただし、コマンドの継続性を確保するために、モードが変更された場合でも、新しい入力コマンドのトピックが届くまでこれらのコマンドは変更されません。


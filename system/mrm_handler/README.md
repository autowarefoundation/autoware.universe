# mrm_handler

## 目的

MRM Handlerは、OperationModeAvailabilityに含まれるシステム障害状態から適切なMRMを選択するノードです。

## 内部動作 / アルゴリズム

### 状態遷移

![mrm-state](image/mrm-state.svg)

## 入出力

### 入力

| 名称                                   | 種別                                                | 説明                                                                                           |
| -------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `/localization/kinematic_state`        | `nav_msgs::msg::Odometry`                           | 車両が停止しているかどうか判断するために使用                                                  |
| `/system/operation_mode/availability`  | `tier4_system_msgs::msg::OperationModeAvailability` | operationModeAvailabilityに示されている適切なMRMをシステム利用可能なMRM動作から選択するために使用 |
| `/vehicle/status/control_mode`         | `autoware_vehicle_msgs::msg::ControlModeReport`     | 車両モード（自律運転または手動運転）をチェックするために使用                                |
| `/system/mrm/emergency_stop/status`    | `tier4_system_msgs::msg::MrmBehaviorStatus`         | MRM緊急停止動作が利用可能かどうかをチェックするために使用                                    |
| `/system/mrm/comfortable_stop/status`  | `tier4_system_msgs::msg::MrmBehaviorStatus`         | MRM快適停止動作が利用可能かどうかをチェックするために使用                                  |
| `/system/mrm/pull_over_manager/status` | `tier4_system_msgs::msg::MrmBehaviorStatus`         | MRMプルオーバー動作が利用可能かどうかをチェックするために使用                               |
| `/api/operation_mode/state`            | `autoware_adapi_v1_msgs::msg::OperationModeState`   | 現在の動作モードがAUTOまたはSTOPであるかどうかを確認するために使用                            |

### 出力

**自己位置推定（Self-Localization）**

自己位置推定は、'post resampling`を実行する前に、周囲の地図に対して車両の現在の位置（'current pose`）を推定します。これにより、正確な自己位置を確保し、周囲の障害物を適切に把握できます。

**パスプランニング（Path Planning）**

パスプランニングでは、現時点での自車位置から目標地点まで移動する最適なパスを生成します。障害物や交通状況などの情報を考慮して、安全で効率的な経路を特定します。

**モーションプランニング（Motion Planning）**

モーションプランニングは、生成されたパスに従って、車両の運動を制御します。加速度、速度などの車両の動作を決定し、障害物との衝突を回避する安全な動作を確保します。

**障害物検出（Object Detection）**

障害物検出は、周囲の環境を監視し、車両の周囲にある静止または移動中の障害物を特定します。これにより、衝突の回避と安全な動作を確保できます。

**センサー融合（Sensor Fusion）**

センサー融合は、レーダー、LiDAR、カメラなどの複数のセンサーからのデータを統合し、より正確で完全な周囲環境の認識を行います。これにより、車両が周囲の状況をより適切に把握できます。

**システム統合（System Integration）**

システム統合は、自己位置推定、パスプランニング、モーションプランニング、障害物検出、センサー融合などのさまざまなコンポーネントを組み合わせ、統一された自律走行システムを作成します。この統合により、安全で効率的な自律走行が可能になります。

**Autoware**

Autowareは、オープンソースの自律走行ソフトウェアプラットフォームであり、自己位置推定、パスプランニング、モーションプランニング、障害物検出などのコンポーネントを含む、自律走行に必要な機能を包括的に提供しています。

**パフォーマンス評価**

パフォーマンス評価では、自律走行システムの安全性、効率性、信頼性を測定します。これには、車両の動作、障害物の回避、交通状況への適応などの指標が含まれます。

**セーフティ（Safety）**

安全は自律走行システムの最優先事項です。システムは障害物の検出、衝突の回避、安全な動作を確保するように設計されています。

**効率性（Efficiency）**

効率性は、自律走行システムが目的地に安全かつ迅速に到着するために重要です。システムは、最適なパスを生成し、適切な速度を決定し、エネルギー消費を最小化するように設計されています。

**信頼性（Reliability）**

信頼性は、自律走行システムがさまざまな状況下で予測可能かつ一貫して動作するために重要です。システムは、センサーの故障、天候の変化、交通状況の変化に耐えられるように設計されています。

| 名前                                    | 種類                                              | 説明                                           |
| --------------------------------------- | ------------------------------------------------- | ----------------------------------------------------- |
| `/system/emergency/gear_cmd`            | `autoware_vehicle_msgs::msg::GearCommand`         | MRMの適切な実行に必要な（ギアコマンドを送信）        |
| `/system/emergency/hazard_lights_cmd`   | `autoware_vehicle_msgs::msg::HazardLightsCommand` | MRMの適切な実行に必要な（ターンシグナルコマンドを送信） |
| `/system/fail_safe/mrm_state`           | `autoware_adapi_v1_msgs::msg::MrmState`           | MRM実行状態と選択されたMRM挙動を通知          |
| `/system/mrm/emergency_stop/operate`    | `tier4_system_msgs::srv::OperateMrm`              | MRM緊急停止用実行命令                              |
| `/system/mrm/comfortable_stop/operate`  | `tier4_system_msgs::srv::OperateMrm`              | MRM快適停止用実行命令                              |
| `/system/mrm/pull_over_manager/operate` | `tier4_system_msgs::srv::OperateMrm`              | MRM停車用実行命令                                 |

## パラメータ

{{ json_to_markdown("system/mrm_handler/schema/mrm_handler.schema.json") }}

## 前提条件 / 制限事項

未定


# tier4_state_rviz_plugin

## 目的

このプラグインはAutowareの現在の状態を表示します。
また、プラグインはパネルからも起動できます。

## 入出力

### 入力

| 名前                                    | タイプ                                                        | 説明                                                   |
| -------------------------------------- | --------------------------------------------------------------- | ------------------------------------------------------------- |
| `/api/operation_mode/state`            | `autoware_adapi_v1_msgs::msg::OperationModeState`            | オペレーションモードの状態を表します                   |
| `/api/routing/state`                   | `autoware_adapi_v1_msgs::msg::RouteState`                   | ルートの状態を表します                                |
| `/api/localization/initialization_state` | `autoware_adapi_v1_msgs::msg::LocalizationInitializationState` | ローカリゼーション初期化の状態を表します |
| `/api/motion/state`                    | `autoware_adapi_v1_msgs::msg::MotionState`                   | モーションの状態を表します                                |
| `/api/autoware/get/emergency`          | `tier4_external_api_msgs::msg::Emergency`                   | 外部緊急事態の状態を表します                           |
| `/vehicle/status/gear_status`          | `autoware_vehicle_msgs::msg::GearReport`                    | ギアの状態を表します                                    |

### 出力

**自動運転ソフトウエア**

**はじめに**

本ドキュメントでは、自動運転車両向けに開発されたオープンソースソフトウェアAutowareのアーキテクチャと実装について説明する。Autowareは、障害物検出や経路計画などの運転タスクを実行するためのモジュール化されたコンポーネントの集合である。

**アーキテクチャ**

Autowareアーキテクチャは、Perception、Planning、Controlの3層構造となっている。

**Perception**

Perception層は、センサーデータから周囲環境を認識する。LIDAR、カメラ、レーダーなどのセンサーからのデータを生成し、オブジェクトを検出し、分類し、追跡する。

**Planning**

Planning層は、車両の経路を計画する。周囲環境の認識情報と自車位置に基づき、「post resampling」手法を使用して経路を生成する。経路は、速度、加速度、制御逸脱量などの制約条件が考慮される。

**Control**

Control層は、計画された経路に基づいて車両を制御する。この層は、ステアリング、アクセル、ブレーキを制御し、安全に軌道を追従する。

**モジュール**

Autowareは、次のような主要モジュールで構成されている。

* **Perception:**
    * LIDAR Point Cloud Filter
    * Object Detection
    * Object Tracking
* **Planning:**
    * Path Planning
    * Trajectory Generation
* **Control:**
    * Motion Planner
    * Vehicle Control

**実装**

Autowareは、ROS（Robot Operating System）を使用して実装されている。モジュール間の通信は、トピックとサービスを通じて行われる。Autowareには、シミュレーションやテスト用のさまざまなツールも含まれている。

**使用法**

Autowareは、自動運転車両の開発に使用できる。研究目的でも、商用製品の開発でも使用できる。Autowareはオープンソースであり、GitHubで利用できる。

**追加資料**

* [Autoware GitHubリポジトリ](https://github.com/autowarefoundation/autoware.auto)
* [Autowareドキュメント](https://www.autoware.org/documentation/)

| 名前                                                 | タイプ                                                 | 説明                                                 |
| ---------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------------------- |
| `/api/operation_mode/change_to_autonomous`         | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | 自動運転モードに変更するサービス                      |
| `/api/operation_mode/change_to_stop`               | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | 停止モードに変更するサービス                        |
| `/api/operation_mode/change_to_local`              | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | ローカルモードに変更するサービス                      |
| `/api/operation_mode/change_to_remote`             | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | リモートモードに変更するサービス                      |
| `/api/operation_mode/enable_autoware_control`      | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | Autowareによる車両制御を有効にするサービス            |
| `/api/operation_mode/disable_autoware_control`     | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | Autowareによる車両制御を無効にするサービス            |
| `/api/routing/clear_route`                         | `autoware_adapi_v1_msgs::srv::ClearRoute`          | ルート状態をクリアするサービス                        |
| `/api/motion/accept_start`                         | `autoware_adapi_v1_msgs::srv::AcceptStart`         | 車両の始動を受け入れるサービス                      |
| `/api/autoware/set/emergency`                      | `tier4_external_api_msgs::srv::SetEmergency`       | 外部緊急状態を設定するサービス                        |
| `/planning/scenario_planning/max_velocity_default` | `tier4_planning_msgs::msg::VelocityLimit`          | 車両の最大速度を設定するトピック                      |

## 使用方法

1. rvizを起動し、パネル/新しいパネルを追加を選択する。

   ![select_panel](./images/select_panels_ja.png)

2. tier4_state_rviz_plugin/AutowareStatePanelを選択し、OKを押す。

   ![select_state_plugin](./images/select_state_plugin_ja.png)

3. 自動ボタンがアクティブ化されていれば、クリックでエンゲージできます。

   ![select_auto](./images/select_auto_ja.png)


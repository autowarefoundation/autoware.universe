# 行動経路プランナー

行動経路プランナーの主な目的は、衝突の危険性を最小限に抑えることで自動運転車の安全性を大幅に向上させることです。時間節約による運転効率の向上と、ルールに基づくアプローチによる信頼性の向上を図ります。さらに、ユーザーは独自のカスタム動作モジュールを統合したり、自動車、バス、配送ロボットなどさまざまなタイプの車両や、混雑した都市道路から高速道路までさまざまな環境で使用したりできます。

このモジュールは、自律車両の現在の状況、つまり位置、速度、周囲環境を徹底的に分析することから始まります。この分析により、車線変更や停止に関する重要な運転判断が下され、その後、安全かつ効率的な経路が生成されます。障害物を回避して、他の車両、歩行者、予期しない路上の障害物などの静的および動的障害物に対応しながら、道路形状、交通規則、動的条件を考慮に入れて、安全なナビゲーションを確保します。

さらに、このプランナーは他の交通参加者と積極的に対話し、彼らの行動を予測して、それにしたがって車両の経路を調整します。これにより、自律車両の安全性が確保されるだけでなく、交通の流れが円滑になります。速度制限や交通信号などの交通法規を遵守することで、法に則った予測可能な運転行動がさらに保証されます。このプランナーは、急な旋回や急ブレーキを最小限に抑えるよう設計されており、快適で自然な運転体験を目指しています。

!!! note

    [プランニングコンポーネント設計](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/) ドキュメントは、行動経路プランナーモジュールの設計と将来の開発を導く基本的な哲学の概要を示しています。その現在の構成とその継続的な開発の方向性を理解するために、このドキュメントを参照することを強くお勧めします。

## 目的とユースケース

基本的に、このモジュールには主に3つの責務があります。

1. 交通状況に基づいた経路を作成する。
2. 車両が走行可能な「走行可能領域」を生成する。
3. 車両インターフェイスに中継されるウィンカーコマンドを生成する。

## 機能

### サポートされるシーンモジュール

行動経路プランナーには、次のシーンモジュールがあります。

| 名称                       | 説明                                                                                                                                                                | 詳細                                                                       |
| :------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------- |
| 車線追従             | このモジュールは、レーンレットの中心線から基準パスを生成します。                                                                                                              | [LINK](https://drive.google.com/file/d/1upH4p_9JUod40iEIdAQq4A2H6-52zz-l/view?usp=sharing) |
| 静的障害物回避  | このモジュールは、回避すべきオブジェクトがある場合に回避パスを生成します。                                                                                           | [LINK](https://autoware.readthedocs.io/en/latest/autoware_behavior_path_static_obstacle_avoidance_module/README.md) |
| 動的障害物回避 | WIP                                                                                                                                                                        | [LINK](https://github.com/ros-planning/autoware/tree/master/ros2/autoware_behavior_path_dynamic_obstacle_avoidance_module) |
| 車線変更による回避   | このモジュールは、回避すべきオブジェクトがある場合に車線変更パスを生成します。                                                                                         | [LINK](https://drive.google.com/file/d/1k7x-4T2Yj_bJhp0-IzY12wY4gII_42Vu/view?usp=sharing) |
| 車線変更                | このモジュールは、必要に応じて実行され、他の車両との衝突チェックが完了します。                                                                        | [LINK](https://autoware.readthedocs.io/en/latest/autoware_behavior_path_lane_change_module/README.md) |
| 外部車線変更       | WIP                                                                                                                                                                        | [LINK](https://drive.google.com/file/d/13jDgm9Q7ELR00Gka51B_2XquH6e_61cp/view?usp=sharing) |
| ゴールプランナー               | このモジュールは、自車が道路車線にいるときでゴールが路側車線にいるときに実行されます。自車はゴールで停止します。                                         | [LINK](https://autoware.readthedocs.io/en/latest/autoware_behavior_path_goal_planner_module/README.md) |
| スタートプランナー              | このモジュールは、自車が停止していて、自車のフットプリントが路側車線に含まれているときに実行されます。このモジュールは、自車が道路に合流すると終了します。 | [LINK](https://autoware.readthedocs.io/en/latest/autoware_behavior_path_start_planner_module/README.md) |
| サイドシフト                 | （リモート制御用）外部の命令に従ってパスを左または右にシフトします。                                                                                 | [LINK](https://autoware.readthedocs.io/en/latest/autoware_behavior_path_side_shift_module/README.md) |

!!! Note

    以下の画像をクリックすると、実行中のビデオが表示されます。

    <div align="center">
        <table>
            <tr>
                <td><img src="./image/supported_module_lane_following.svg" alt="車線維持モジュール" width="300"></td>
                <td><a href="https://www.youtube.com/watch?v=A_V9yvfKZ4E"><img src="./image/supported_module_avoidance.svg" alt="回避モジュール" width="300"></a></td>
                <td><img src="./image/supported_module_avoidance_by_lane_change.svg" alt="車線変更による回避モジュール" width="300"></td>
            </tr>
            <tr>
                <td><a href="https://www.youtube.com/watch?v=0jRDGQ84cD4"><img src="./image/supported_module_lane_change.svg" alt="車線変更モジュール" width="300"></a></td>
                <td><a href="https://www.youtube.com/watch?v=xOjnPqoHup4"><img src="./image/supported_module_start_planner.svg" alt="始点プランナーモジュール" width="300"></a></td>
                <td><a href="https://www.youtube.com/watch?v=ornbzkWxRWU"><img src="./image/supported_module_goal_planner.svg" alt="終点プランナーモジュール" width="300"></a></td>
            </tr>
        </table>
    </div>

!!! Note

    Planningコンポーネントの設計については、[Planningコンポーネント設計](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/#supported-functions)を参照してください。

#### 新規モジュールの追加・実装方法

すべてのシーンモジュールは、基本クラス`scene_module_interface.hpp`を継承して実装されます。

!!! Warning

    本項の残りの部分は現在進行中です（WIP）。

### プランナーマネージャー

プランナーマネージャーの責務は次のとおりです。

1. 自動運転車両が直面している特定の状況に応じて、関連するシーンモジュールをアクティブ化します。たとえば、駐車車両によって自車の走行車線がブロックされた場合、マネージャーは回避モジュールを起動します。
2. 複数のモジュールが同時に実行されている場合の実行順序を管理します。たとえば、車線変更モジュールと回避モジュールの両方が稼働している場合、マネージャーがどちらを優先するかを決定します。
3. 複数のモジュールが同時にアクティベートされ、それぞれが独自のパスを生成する場合、それらのパスをマージして単一の機能パスを作成します。

!!! note

    シーンモジュールの遷移（登録済みモジュール、承認済みモジュール、候補モジュール）を確認するには、[動作パスプランナーコンフィグレーションファイル](https://github.com/autowarefoundation/autoware_launch/blob/0cd5d891a36ac34a32a417205905c109f2bafe7b/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml#L3)で`verbose: true`を設定します。

    ![シーンモジュールの遷移表](./image/checking_module_transition.png)

!!! note

    より詳細な情報については、[マネージャー設計](./docs/behavior_path_planner_manager_design.md)ドキュメントを参照してください。

## 入出力/API

### 入力

| 名前                           | 必須? | タイプ                                                                   | 説明                                                                                                                                                                                                                                      |
| :----------------------------- | :-------: | :----------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ~/input/odometry                   |     ○     | `nav_msgs::msg::Odometry`                                     | 自車速度                                                                                                                                                                                                                               |
| ~/input/accel                      |     ○     | `geometry_msgs::msg::AccelWithCovarianceStamped`        | 自車加速度                                                                                                                                                                                                                           |
| ~/input/objects                    |     ○     | `autoware_perception_msgs::msg::PredictedObjects`       | 知覚モジュールからのダイナミックオブジェクト                                                                                                                                                                                      |
| ~/input/occupancy_grid_map         |     ○     | `nav_msgs::msg::OccupancyGrid`                          | 知覚モジュールのオキュパンシグリッドマップ。Goal Plannerモジュールでのみ使用されます。                                                                                                                                                     |
| ~/input/traffic_signals            |     ○     | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 知覚モジュールからの交通信号情報                                                                                                                                                                                                   |
| ~/input/vector_map                 |     ○     | `autoware_map_msgs::msg::LaneletMapBin`                 | ベクタマップ情報                                                                                                                                                                                                                        |
| ~/input/route                    |     ○     | `autoware_planning_msgs::msg::LaneletRoute`             | スタートからゴールまでの現在のルート                                                                                                                                                                                                   |
| ~/input/scenario                 |     ○     | `tier4_planning_msgs::msg::Scenario`                    | 現在シナリオが`Scenario:LaneDriving`の場合、ビヘイビアパスプランナーを起動                                                                                                                                                            |
| ~/input/lateral_offset           |     △     | `tier4_planning_msgs::msg::LateralOffset`               | サイドシフトをトリガするための横方向オフセット                                                                                                                                                                                              |
| ~/system/operation_mode/state      |     ○     | `autoware_adapi_v1_msgs::msg::OperationModeState`       | 車両が自律モードにあるか、制御可能であるかどうかをPlanningモジュールに伝える<sup>[参照](https://github.com/autowarefoundation/autoware.universe/blob/main/system/autoware_default_adapi/document/operation-mode.md)</sup> |

- ○ 必須: このどれか1つでも存在しない場合、Planning Moduleは動作しません。
- △ オプション: 一部のモジュールは動作しませんが、Planning Moduleは引き続き動作できます。

### 出力

| 名前 | タイプ | 説明 | QoSの耐久性 |
|---|---|---|---|
| ~/output/path | `tier4_planning_msgs::msg::PathWithLaneId` | 各モジュールによって生成された経路 | `volatile` |
| ~/output/turn_indicators_cmd | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | ターンシグナルのコマンド | `volatile` |
| ~/output/hazard_lights_cmd | `autoware_vehicle_msgs::msg::HazardLightsCommand` | ハザードランプのコマンド | `volatile` |
| ~/output/modified_goal | `autoware_planning_msgs::msg::PoseWithUuidStamped` | 変更された目標コマンドの出力を示す | `transient_local` |
| ~/output/stop_reasons | `tier4_planning_msgs::msg::StopReasonArray` | 自車停止の理由を説明 | `volatile` |
| ~/output/reroute_availability | `tier4_planning_msgs::msg::RerouteAvailability` | モジュールが採用しようとしている経路。外部からの承認が得られ次第実行される | `volatile` |

### デバッグ

#### 障害物検出（障害物検出モジュール）

##### 障害物検出の可視化

1. Rvizで `ObstacleDetection` トピックを subscribe します。
2. `rviz_visual_tools` で **Object** マーカーを有効にします。
   - `MarkerFrame`: `map`
   - `MarkerTopic`: `ObstacleDetection`
   - `MarkerShape`: `Sphere`

##### デバッグコマンド

- `/obstacle_detection/debug_info` トピックを subscribe し、障害物検出の結果を確認します。
- `/obstacle_detection/obstacles` トピックを subscribe し、障害物の情報を取得します。
- `/obstacle_detection/cluster` トピックを subscribe し、クラスタリングされた障害物の情報を取得します。

#### 行動計画（Planning）

##### 行動計画の可視化

1. Rvizで `Planning` トピックを subscribe します。
2. Rvizで **Path** マーカーを有効にします。
   - `MarkerFrame`: `map`
   - `MarkerTopic`: `Planning`
   - `MarkerShape`: `Arrow`

##### デバッグコマンド

- `/planning/debug_info` トピックを subscribe し、行動計画の結果を確認します。
- `/planning/trajectory` トピックを subscribe し、期待される経路を取得します。

#### 制御（Control）

##### `post resampling` コントローラーの可視化

1. Rvizで `Control` トピックを subscribe します。
2. Rvizで **Path** マーカーを有効にします。
   - `MarkerFrame`: `map`
   - `MarkerTopic`: `Control`
   - `MarkerShape`: `Arrow`

##### デバッグコマンド

- `/control/debug_info` トピックを subscribe し、制御の結果を確認します。
- `/control/trajectory` トピックを subscribe し、制御された経路を取得します。

#### ローカリゼーション

##### 自車位置の可視化

1. Rvizで `Localization` トピックを subscribe します。
2. Rvizで **Pose** マーカーを有効にします。
   - `MarkerFrame`: `map`
   - `MarkerTopic`: `Localization`
   - `MarkerShape`: `Cube`

##### デバッグコマンド

- `/localization/debug_info` トピックを subscribe し、ローカリゼーションの結果を確認します。
- `/localization/pose` トピックを subscribe し、自車位置を取得します。

| 名前 | タイプ | 説明 | QoS の永続性 |
|---|---|---|---|
| ~/debug/avoidance_debug_message_array | `tier4_planning_msgs::msg::AvoidanceDebugMsgArray` | Avoidance のデバッグメッセージ。Avoidance パスを生成できない理由をユーザーに通知します | `volatile` |
| ~/debug/lane_change_debug_message_array | `tier4_planning_msgs::msg::LaneChangeDebugMsgArray` | レーン変更のデバッグメッセージ。レーン変更処理中の危険な理由をユーザーに通知します | `volatile` |
| ~/debug/maximum_drivable_area | `visualization_msgs::msg::MarkerArray` | 最大の静的走行可能領域を表示します | `volatile` |
| ~/debug/turn_signal_info | `visualization_msgs::msg::MarkerArray` | TBA | `volatile` |
| ~/debug/bound | `visualization_msgs::msg::MarkerArray` | 静的走行可能領域のデバッグ | `volatile` |
| ~/planning/path_candidate/* | `autoware_planning_msgs::msg::Path` | 承認前のパス | `volatile` |
| ~/planning/path_reference/* | `autoware_planning_msgs::msg::Path` | 各モジュールによって生成されたリファレンスパス | `volatile` |

!!! note

    トピックのサブスクライブと発行に関する具体的な情報は、[behavior_path_planner.xml](https://github.com/autowarefoundation/autoware.universe/blob/9000f430c937764c14e43109539302f1f878ed70/planning/behavior_path_planner/launch/behavior_path_planner.launch.xml#L36-L49) を参照してください。

## モジュールの有効化と無効化の方法

Behavior Path Planning内のモジュールの有効化と無効化は、主に2つの主要なファイル(`default_preset.yaml`と`behavior_path_planner.launch.xml`)によって管理されます。

`default_preset.yaml`ファイルは、プランナー内の特定のモジュールを有効化または無効化するための設定ファイルとして機能します。Behavior Path Planningのモジュールまたは機能を表す一連の引数を含みます。例:

- `launch_static_obstacle_avoidance_module`: 回避モジュールを有効にするには`true`に、無効にするには`false`に設定します。

!!! note

    `default_preset.yaml`を表示するには、[ここ](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/preset/default_preset.yaml)をクリックします。

`behavior_path_planner.launch.xml`ファイルは、`default_preset.yaml`に定義された設定を参照して、Behavior Path Planningのノード実行時に設定を適用するために使用します。たとえば、`behavior_path_planner.launch.xml`の`static_obstacle_avoidance.enable_module`パラメータでは、


```xml
<param name="static_obstacle_avoidance.enable_module" value="$(var launch_static_obstacle_avoidance_module)"/>
```

### `default_preset.yaml`の`launch_static_obstacle_avoidance_module`に対応する。

したがって、モジュールを有効または無効にするには、`default_preset.yaml`内の対応するモジュールを`true`または`false`に設定します。これらの変更は、Autowareの次回の起動時に適用されます。

## パス生成

精緻な手法がパスの生成に使用され、特に車線変更や回避などの操作に焦点を当てます。この設計の中核は、一定の加速度変化率によって達成される、基準パスのスムーズな横方向シフトです。このアプローチにより、加速度の変化率が一定になり、スムーズな遷移が促進され、横方向動特性の突然の変化が最小限に抑えられます。これは乗客の快適さと安全に不可欠です。

この設計には、時間の経過に伴う車両パスの横方向のシフトを計算するための複雑な数学的定式化が含まれます。これらの計算には、車両の横方向の変位、速度、および加速度の決定、車両の横方向の加速度と速度の制限の考慮が含まれます。これにより、車両の動きが安全かつ管理可能であることが保証されます。

`ShiftLine`構造体（[こちら](https://github.com/autowarefoundation/autoware.universe/blob/9000f430c937764c14e43109539302f1f878ed70/planning/behavior_path_planner/include/behavior_path_planner/utils/path_shifter/path_shifter.hpp#L35-L48)に記載）は、横方向シフトの開始点と終了点を表すパス上のポイントを表すために使用されます。これには、絶対座標系での開始点と終了点、基準パスと比較したこれらのポイントでの相対シフトの長さ、および基準パス上の対応するインデックスなどの詳細が含まれます。この構造体はパスシフトを管理するのに不可欠であり、パスプランナーが車両の現在の位置と計画されている操作に基づいて軌道を動的に調整できます。

さらに、この設計とその実装には、パスシフトに必須のパラメータを計算するためのさまざまな方程式と数学的モデルが組み込まれています。これらには、横方向シフトの総距離、許容される最大横方向加速度とジャーク、シフトに必要な総時間が含まれます。また、ほとんどの車線変更と回避のケースでは特定のタイムインターバルがないときに仮定を単純化するなど、実際的な考慮事項にも注意します。

シフトパス生成ロジックにより、ビヘイビアパスプランナーは安全かつ効率的なパスを動的に生成し、横方向の動きを正確に制御して、車線変更と回避操作の円滑な実行を保証できます。この慎重な計画と実行は、車両の動的機能と安全上の制約を順守し、自律車両ナビゲーションにおける効率と安全性を最大化します。

!!! note

    数学好きの方は、詳細については[パス生成設計](../autoware_behavior_path_planner_common/docs/behavior_path_planner_path_generation_design.md)を参照してください。

## 衝突評価/安全チェック

ビヘイビアパスプランナーの衝突評価機能の目的は、すべてのモジュールを横断してターゲットオブジェクトとの衝突の可能性を評価することです。2つのシナリオで使用されます。

1. 候補パスの生成中、生成された候補パスが衝突のないものであることを確認します。
2. パスがマネージャーによって承認され、自己車両が現在のモジュールを実行しているとき、現在の状況が安全でない場合は、各モジュールの要件に応じて、プランナーは実行をキャンセルするか、別のモジュールを実行することを選択します。

安全チェックプロセスはいくつかのステップを含みます。最初に、通常予測パスを補間して、特定の時点でのターゲットオブジェクトの位置を取得します。次に、この時点で自己車両とターゲットオブジェクトとの間にオーバーラップがないか確認します。オーバーラップが検出された場合、パスは安全ではないと判断されます。また、関数の与えられたパスに沿った弧長を使用して、どちらの車両が前方にいるかを識別します。この関数は、自己車両（自律車両）とターゲットオブジェクトの両方について、正確な位置、速度、形状のデータが利用可能であるという前提で動作します。また、これらのオブジェクトの予測パス内の各点のヨー角に依存し、次のパス点の方向を向くことが想定されています。

安全チェックの重要な部分は、RSS（責任感応型安全）距離に着想を得たアルゴリズムの計算です。このアルゴリズムは、反応時間、安全時間マージン、両方の車両の速度と減速などの要因を考慮します。自己車両とターゲット車両の両方に、拡張オブジェクトポリゴンが作成されます。特に、後方のオブジェクトのポリゴンは、縦方向にRSS距離、横方向にマージンで拡張されます。最後に、この拡張された後方のオブジェクトのポリゴンと前方オブジェクトのポリゴンとのオーバーラップをチェックします。オーバーラップは、潜在的に安全でない状況を示します。

ただし、モジュールにはターゲットオブジェクトの予測パス内の各点のヨー角に関する制限があり、常に次の点に向いていない場合があります。これにより、エッジケースで潜在的な不正確さが発生する場合があります。

!!! note

    衝突評価手法の詳細については、[安全チェックユーティリティ](../autoware_behavior_path_planner_common/docs/behavior_path_planner_safety_check.md)を参照してください。

## 走行可能領域の生成

### 静的走行可能領域のロジック

走行可能領域は、自己車両が走行できる領域を決定するために使用されます。静的走行可能領域の拡張の主な目的は、車両の現在の動作に必要なスペースのみを含む領域を生成することによって安全な走行を確保することですが、不要な領域を除外します。たとえば、`回避`モジュールが実行されているとき、走行可能領域には障害物を回避するための操作に必要な追加のスペースが含まれ、動作はレーンレットの領域の外側に回避パスを拡張しないことで制限されます。

<div align="center">
    <table>
        <tr>
            <td><img src="./image/static_drivable_area_before_expansion.png" alt="Before expansion"></td>
        </tr>
        <tr>
            <td><img src="./image/static_drivable_area_after_expansion.png" alt="After expansion"></td>
        </tr>
    </table>
</div>

静的走行可能領域の拡張は、レーンの正しい配置と、左端と右端の境界内での車両の前部と後部の両方の範囲について、次の動作で行われます:

走行可能領域を生成するための重要なパラメータとしては、自己車両の追加フットプリントオフセット、動的オブジェクトの処理、最大拡張距離、および拡張のための具体的な方法があります。さらに、各モジュールは独自の走行可能領域を生成するため、次の実行モジュールの走行可能領域を生成するための入力として渡すか、統一された走行可能領域を生成する前に、車両の走行順序に基づいて走行可能レーンのソートを行います。これにより、走行可能領域の生成に使用されるレーンの正しい定義が確保されます。

!!! note

### Dynamic Drivable Area Logic

大型車両の旋回にはより広い空間が必要で、現在の車線を逸脱することがあります。例えば、交差点でバスが曲がっている場合です。そのような場合、静的な走行可能領域に依存することは不十分です。静的な方法は、高精細マップによって提供される車線情報に依存するためです。静的なアプローチの制限を克服するために、動的走行可能領域の拡張アルゴリズムは、自律車両の通行可能なスペースをリアルタイムで調整します。以前に計算された経路データを再利用することで計算処理を節約し、車両の位置に大きな変化がある場合にのみ更新します。システムは、車両の旋回半径や他の動的要因に対応するために必要な最小車線幅を評価します。その後、車両の経路曲率を考慮して、安全な走行のための十分なスペースを確保するために必要な走行可能領域の境界の最適な拡張を計算します。これらの境界を拡張または縮小できる速度は、車両のナビゲーションの安定性を維持するために調整されています。このアルゴリズムは、固定障害物を回避し、法的な走行制限に従いながら、走行可能なスペースを最大化することを目的としています。最後に、これらの境界線の調整を適用し、経路曲率の計算を滑らかにすることで、車両の運用を通じて安全で法令に準拠した通行可能な経路が維持されるようにします。

!!! note

    この機能は [drivable_area_expansion.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/0cd5d891a36ac34a32a417205905c109f2bafe7b/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml#L10) で有効にすることができます。

## ターンシグナルの生成

Behavior Path Plannerモジュールは、ターンシグナルのコマンドを出力するために `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` を使用します（[TurnIndicatorsCommand.idl](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/TurnIndicatorsCommand.msg) を参照）。システムは運転状況を評価し、旋回、車線変更、障害物回避などの走行計画に基づいて、いつターンシグナルを作動させるかを決定します。

このフレームワークの中で、システムは**必要な**点滅器作動と**不要な**点滅器作動を区別します。**不要な**作動は、車線変更や旋回の前の合図など、一般的な運転シナリオで交通法によって推奨されるものです。**必要な**作動は、障害物を避けるために急な車線変更を合図するなど、安全上の理由から必須とみなされるものです。

`TurnIndicatorsCommand` メッセージ構造には、いくつかの定数のいずれかを保持できるコマンドフィールドがあります。`NO_COMMAND` は信号が不要であることを示し、`DISABLE` は信号を無効にし、`ENABLE_LEFT` は左折を合図し、`ENABLE_RIGHT` は右折を合図します。Behavior Path Planner は、点滅器作動の**不要**および**必要**の両方のシナリオを考慮するルールベースのシステムに基づいて、適切なタイミングでこれらのコマンドを送信します。

!!! note

    詳細については、[ターンシグナル設計](../autoware_behavior_path_planner_common/docs/behavior_path_planner_turn_signal_design.md)ドキュメントを参照してください。

## リルーティング

!!! warning

    リルーティングは現在進行中の機能です。詳細情報については、後日掲載いたします。

## パラメータと構成

[設定ファイル](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner) は、操作と管理の容易さのために階層的なディレクトリ構造で整理されています。各サブディレクトリには、そのモジュールに関連する具体的な構成ファイルが含まれています。ルートディレクトリには、プランナー全体の動作に適用される一般的な構成ファイルがあります。以下は、ディレクトリ構造とその構成ファイルの概要です。


```text
behavior_path_planner
├── behavior_path_planner.param.yaml
├── drivable_area_expansion.param.yaml
├── scene_module_manager.param.yaml
├── static_obstacle_avoidance
│   └── static_obstacle_avoidance.param.yaml
├── avoidance_by_lc
│   └── avoidance_by_lc.param.yaml
├── dynamic_obstacle_avoidance
│   └── dynamic_obstacle_avoidance.param.yaml
├── goal_planner
│   └── goal_planner.param.yaml
├── lane_change
│   └── lane_change.param.yaml
├── side_shift
│   └── side_shift.param.yaml
└── start_planner
    └── start_planner.param.yaml
```

同様に、[common](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning/scenario_planning/common) ディレクトリには、さまざまなモジュールで使用される設定ファイルが含まれています。これらの設定ファイルは、Behavior Path Planner の機能に不可欠な共通パラメータと設定を提供します。


```text
common
├── common.param.yaml
├── costmap_generator.param.yaml
└── nearest_search.param.yaml
```

[Preset](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning/preset) ディレクトリには、さまざまなモジュールの動作状態を管理するための構成が含まれています。これには、システム内のモジュールの有効化や無効化を具体的に処理する `default_preset.yaml` ファイルが含まれます。


```text
preset
└── default_preset.yaml
```

## 制限事項と今後の取り組み

1. ゴールプランナーモジュールは、他のモジュールと同時に実行できません。
2. このモジュールはプラグインとして設計されていません。カスタムモジュールの統合は容易ではなく、ユーザーは動作パス計画メインコードの一部を変更する必要があります。


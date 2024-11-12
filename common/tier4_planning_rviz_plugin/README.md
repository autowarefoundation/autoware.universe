## tier4_planning_rviz_plugin

このパッケージにはjskコードが含まれます。
jsk_overlay_utils.cppとjsk_overlay_utils.hppはBSDライセンスです。

## 目的

このプラグインは、パス、軌跡、および最高速度を表示します。

## 入出力

### 入力
- current pose
- velocity profile
- planned trajectory
- v\_max

### Output
- No outputs

## Usage

```roslaunch tier4_planning_rviz_plugin tier4_planning_rviz_plugin.launch [args...]```

## Arguments

```
--topic        /planning/trajectory
--pose_topic   /current_pose
--velocty_topic  /planning/velocity_profile
--max_velo      /planning/v_max
--frame_id     base_link
--max_lateral_jerk  0.5
--max_longitudinal_jerk  1.0
```

| 名前                                           | タイプ                                      | 説明                                        |
| --------------------------------------------- | ----------------------------------------- | ------------------------------------------- |
| `/input/path`                                 | `autoware_planning_msgs::msg::Path`       | パスをサブスクライブするトピック            |
| `/input/trajectory`                             | `autoware_planning_msgs::msg::Trajectory` | トラジェクトリをサブスクライブするトピック |
| `/planning/scenario_planning/current_max_velocity` | `tier4_planning_msgs/msg/VelocityLimit`   | 最大速度をパブリッシュするトピック           |

### 出力

このドキュメントでは、AutowareにおけるPlanningコンポーネント内の経路計画モジュールの詳細を説明します。

#### 目的

経路計画モジュールは、障害物や走行制限を考慮した、安全で効率的な経路を生成します。

#### 入力

* 自車位置（`current pose`）
* 目的地
* 障害物マップ
* 走行制限マップ

#### 出力

* 経路（`post resampling`）
* 速度プロファイル
* 加速度逸脱量
* 速度逸脱量

#### アルゴリズム

経路計画モジュールは、以下を含む複数のサブコンポーネントを使用します。

* **経路探索:** A*アルゴリズムを使用して、障害物を回避する最適な経路を探索します。
* **経路平滑化:** 経路の急カーブや急激な速度変化を滑らかにします。
* **速度計画:** 速度制限や加速制限を考慮して、安全な速度プロファイルを作成します。

#### 注意事項

* 経路計画モジュールは、障害物マップと走行制限マップの正確性に依存します。
* モジュールは動的環境では動作しません。
* 経路は**推奨経路**であることに注意してください。実際の経路は、センサーデータや他の制御システムによって修正される可能性があります。

| 名称                                    | 種別                           | 説明                                      |
| --------------------------------------- | ------------------------------ | ----------------------------------------- |
| `/planning/mission_planning/checkpoint` | `geometry_msgs/msg/PoseStamped` | チェックポイントを公開するトピック       |

## パラメーター

### コアパラメーター

#### MissionCheckpoint

| 名前 | 型 | 初期値 | 説明 |
|---|---|---|---|
| `pose_topic_property_` | 文字列 | `mission_checkpoint` | チェックポイントを発行するトピック |
| `std_dev_x_` | 浮動小数 | 0.5 | チェックポイント姿勢の X 標準偏差 [m] |
| `std_dev_y_` | 浮動小数 | 0.5 | チェックポイント姿勢の Y 標準偏差 [m] |
| `std_dev_theta_` | 浮動小数 | M_PI / 12.0 | チェックポイント姿勢の Theta 標準偏差 [rad] |
| `position_z_` | 浮動小数 | 0.0 | チェックポイント姿勢の Z 位置 [m] |

#### Path

**Planning** コンポーネントは、現在の経路の経路を生成し、各経路ポイントの速度と加速度を計算します。経路には、車両の現在の姿勢からゴールまでの道筋を示す一連のセグメントが含まれています。

各セグメントには、以下のパラメータが含まれています。

* 開始と終了の経路ポイント
* 最大速度と加速度
* スプライン補完（オプション）

**Planning** コンポーネントは、以下を考慮して経路を最適化します。

* 交通規制（速度制限、停止線など）
* 交通状況（他の車両、歩行者など）
* 車両の動力学（加速度、減速度、曲がる能力）

最適化された経路が計算されたら、**Planning** コンポーネントはこれを実行される一連のコマンド（コントロールコマンド）に変換します。コントロールコマンドには、車両の速度、加速度、ステアリングの目標値が含まれます。

| 名称                                  | 型     | デフォルト値 | 説明                                                 |
| ------------------------------------ | ------ | ------------- | ---------------------------------------------------- |
| `property_path_view_`                 | bool    | true          | パスプロパティを使用するかどうか                      |
| `property_path_width_view_`           | bool    | false         | 一定の幅を使用するかどうか                            |
| `property_path_width_`                | float   | 2.0           | パスプロパティの幅 [m]                                |
| `property_path_alpha_`                | float   | 1.0           | パスプロパティのアルファ                                |
| `property_path_color_view_`           | bool    | false         | 一定の色を使用するかどうか                            |
| `property_path_color_`                | QColor  | Qt::black     | パスプロパティの色                                    |
| `property_velocity_view_`             | bool    | true          | 速度プロパティを使用するかどうか                      |
| `property_velocity_alpha_`            | float   | 1.0           | 速度プロパティのアルファ                                |
| `property_velocity_scale_`            | float   | 0.3           | 速度プロパティのスケール                                |
| `property_velocity_color_view_`       | bool    | false         | 一定の色を使用するかどうか                            |
| `property_velocity_color_`            | QColor  | Qt::black     | 速度プロパティの色                                    |
| `property_vel_max_`                   | float   | 3.0           | 最大速度 [m/s]                                          |

#### 走行可能領域

このドキュメントでは、Autowareの中核となるコンポーネントであるDrivableAreaについて説明します。

**目的**
DrivableAreaモジュールは、現在の周りの走行可能なエリアを検出し、それをトラジェクトリプランニングとローカルプランニングに提供します。

**入力**
* 自車位置と姿勢
* センサーデータ（点群、画像など）
* Map data (lane information, drivable area information)
* タスクマニュアル（実行するタスク）

**出力**
* 走行可能領域の点群
* 走行可能領域のポリゴン

**アルゴリズム**
DrivableAreaモジュールは、以下のステップで走行可能領域を検出します。

1. **Occupancy Gridの作成:** センサーデータを処理して、周囲環境の占有グリッドを作成します。
2. **点群フィルタリング:** 点群から、地面、障害物、走行不可能なオブジェクトを除去します。
3. **走行可能エリアの抽出:** 除去された点群から、走行可能なエリアを抽出します。
4. **`post resampling`:** 走行可能エリアを滑らかにし、ノイズを除去します。

**パラメータ**
DrivableAreaモジュールの動作は、次のパラメータで構成できます。

* センサーパラメータ（スキャン範囲、解像度など）
* フィルターパラメータ（地面検出しきい値、障害物検出しきい値など）
* 抽出パラメータ（走行可能エリアの最小サイズ、最大傾斜角度など）
* `post resampling`パラメータ（カーネルサイズ、シグマ値など）

**評価指標**
DrivableAreaモジュールの性能は、以下の指標によって評価できます。

* 正答率
* 再現率
* F1スコア
* 処理時間

**依存関係**
DrivableAreaモジュールは、以下のコンポーネントに依存しています。

* ロケーション（自車位置と姿勢の提供）
* センサー（点群と画像データの提供）
* マップ（レーン情報と走行可能エリア情報の提供）
* タスクマネージャー（実行するタスクの提供）

**制限事項**
DrivableAreaモジュールには、次の制限事項があります。

* センサーの死角の影響を受けます。
* 動的障害物（歩行者、車両など）を検出できません。
* 急激な勾配や複雑な環境では、走行可能領域を正確に検出できない場合があります。

| 名前                     | タイプ | 初期値 | 説明                                   |
| ------------------------ | ----- | ------- | --------------------------------------- |
| `color_scheme_property_` | 整数  | 0       | 走行可能エリアプロパティのカラーテーマ |
| `alpha_property_`        | float  | 0.2     | 走行可能エリアプロパティのアルファ値     |
| `draw_under_property_`   | bool   | false   | 背景として描画するか否か                 |

#### PathFootprint

PathFootprintは、トラジェクトリを表示し、その最適解を探索するためのtoolです。これは、Autoware.Planningコンポーネントによって利用されます。

PathFootprintは、次の機能を提供します。

- 複数のトラジェクトリを可視化し、それらを比較するための基準を提供します。
- トラジェクトリ空間を探索し、さまざまなプランニング戦略を評価するための反復可能な方法を提供します。
- PathFootprintは、「post resampling」で生成されたトラジェクトリのコレクションを受け取ります。
- `post resampling`は、トラジェクトリ空間内でランダムなサンプルを生成するMonte Caloベースのアルゴリズムです。
- PathFootprintは、これらのサンプルを評価し、さまざまな基準に基づいてレーティングを行います。
- レーティングは、目標値への近接性、速度/加速度逸脱量、障害物との衝突可能性など、さまざまな要素に基づいています。
- PathFootprintは、最も高い評価を得たトラジェクトリを返します。
- これにより、Autoware.Planningコンポーネントは、自車位置と目標値に基づいて、最適なトラジェクトリを選択できます。

| Name                             | Type   | Default Value | Description                                   |
| -------------------------------- | ------ | ------------- | ---------------------------------------------- |
| `property_path_footprint_view_`  | bool   | true          | パスフットプリントプロパティを使用するかどうかの設定 |
| `property_path_footprint_alpha_` | float  | 1.0           | パスフットプリントプロパティのアルファ値      |
| `property_path_footprint_color_` | QColor | Qt::black     | パスフットプリントプロパティの色             |
| `property_vehicle_length_`       | float  | 4.77          | 車両の長さ [m]                              |
| `property_vehicle_width_`        | float  | 1.83          | 車両の幅 [m]                               |
| `property_rear_overhang_`        | float  | 1.03          | リアオーバーハング [m]                        |

#### Trajectory

AutowareのTrajectoryコンポーネントは、次のことを行う責任があります。
* センサー情報から障害物と車線の認識（perception）。
* 動的経路計画（Dynamic Path Planning）による、自車位置と障害物に関する知識に基づく経路の生成。
* エラストキネティックモデル（Elastokinetic Model）による、経路に対する速度、加速度制御。

経路の生成プロセスは、2つの主要なサブプロセスに分かれています。

1. **グローバルパスプランニング**：車線情報に基づいて可能な経路を計算します。
2. **ローカルパスプランニング**：障害物と自車位置に基づいて現実的な経路を計算します。

ローカルパスプランニングでは、以下を考慮して経路を計算します。
* 障害物逸脱量
* 加速度逸脱量
* 速度逸脱量

また、経路の生成時には以下の制約が考慮されます。
* 最小曲率半径
* 最大速度
* 最大加速度

経路が生成されると、'post resampling`が適用され、Planningコンポーネントに渡される前に、経路が滑らかで連続的になります。

| 名称                                   | 型      | 初期値      | 説明                                                |
| --------------------------------------- | -------- | ----------- | ----------------------------------------------------- |
| `property_path_view_`                   | 論理値   | true         | パスプロパティを使用するかどうか                       |
| `property_path_width_`                  | 浮動小数 | 2.0          | パスプロパティの幅 [m]                               |
| `property_path_alpha_`                  | 浮動小数 | 1.0          | パスプロパティのアルファ                               |
| `property_path_color_view_`             | 論理値   | false        | 定数値を使用するかどうか                             |
| `property_path_color_`                  | QColor    | Qt::black    | パスプロパティの色                                   |
| `property_velocity_view_`               | 論理値   | true         | Velocityプロパティを使用するかどうか               |
| `property_velocity_alpha_`              | 浮動小数 | 1.0          | Velocityプロパティのアルファ                           |
| `property_velocity_scale_`              | 浮動小数 | 0.3          | Velocityプロパティのスケール                           |
| `property_velocity_color_view_`         | 論理値   | false        | 定数値を使用するかどうか                             |
| `property_velocity_color_`              | QColor    | Qt::black    | Velocityプロパティの色                               |
| `property_velocity_text_view_`          | 論理値   | false        | Velocityをテキストで表示する                          |
| `property_velocity_text_scale_`         | 浮動小数 | 0.3          | Velocityプロパティのスケール                           |
| `property_vel_max_`                     | 浮動小数 | 3.0          | 最大速度 [m/s]                                        |

#### TrajectoryFootprint

このコンポーネントは、受け取った経路を処理し、経路上の每个のポイントに経路幅と経路半径を追加します。これにより、経路上に占有スペースを生成し、経路追従時に空間の制約を維持できます。

このコンポーネントは次の入力を取ります。

* `CurrentPose`：自車位置
* `LaneCenterline`：車線の中心線
* `Trajectory`：プランニングコンポーネントから提供された経路

そして次の出力を生成します：

* `TrajectoryFootprint`：経路幅と経路半径が追加された経路

| 名前                                  | タイプ | デフォルト値              | 説明                                  |
| -------------------------------------- | ------ | -------------------------- | ----------------------------------------- |
| `property_trajectory_footprint_view_`  | bool   | true                       | Trajectory Footprint プロパティを使用するかどうか |
| `property_trajectory_footprint_alpha_` | float  | 1.0                        | Trajectory Footprint プロパティのアルファ値   |
| `property_trajectory_footprint_color_` | QColor | QColor(230, 230, 50)       | Trajectory Footprint プロパティのカラー値   |
| `property_vehicle_length_`             | float  | 4.77                       | 車両の長さ [m]                          |
| `property_vehicle_width_`              | float  | 1.83                       | 車両の幅 [m]                            |
| `property_rear_overhang_`              | float  | 1.03                       | リアオーバーハング [m]                      |
| `property_trajectory_point_view_`      | bool   | false                      | Trajectory Point プロパティを使用するかどうか |
| `property_trajectory_point_alpha_`     | float  | 1.0                        | Trajectory Point プロパティのアルファ値       |
| `property_trajectory_point_color_`     | QColor | QColor(0, 60, 255)         | Trajectory Point プロパティのカラー値       |
| `property_trajectory_point_radius_`    | float  | 0.1                        | Trajectory Point プロパティの半径値          |

#### MaxVelocity

Planningモジュールによって計算された最も速く走行可能な速度。この速度は、周囲環境や自車位置に基づいてシステムが決定します。

| 名前                    | タイプ   | デフォルト値                                      | 説明                                            |
| ----------------------- | ------ | -------------------------------------------------- | ------------------------------------------------ |
| `property_topic_name_`  | 文字列 | `/planning/scenario_planning/current_max_velocity` | 最大速度を購読するためのトピック              |
| `property_text_color_`  | QColor | QColor(255, 255, 255)                              | テキストの色                                    |
| `property_left_`        | 整数    | 128                                                | プロッタウィンドウの左側 [px]                |
| `property_top_`         | 整数    | 128                                                | プロッタウィンドウの上 [px]                  |
| `property_length_`      | 整数    | 96                                                 | プロッタウィンドウの長さ [px]                |
| `property_value_scale_` | 浮動小数 | 1.0 / 4.0                                          | 値のスケール                                    |

## 使用方法

1. rvizを起動し、[表示]パネルで[追加]を選択します。
   ![select_add](./images/select_add.png)
2. tier4_planning_rviz_pluginのいずれかを選択して、[OK]を押します。
   ![select_planning_plugin](./images/select_planning_plugin.png)
3. パスまたは軌跡を表示するトピックの名前を入力します。
   ![select_topic_name](./images/select_topic_name.png)

## Material Designアイコン

このプロジェクトでは、Googleによる[Material Designアイコン](https://developers.google.com/fonts/docs/material_symbols)を使用しています。これらのアイコンは、Apache Licenseバージョン2.0の条件に基づいて使用されています。

Material Designアイコンは、Googleが提供する記号の集まりで、アプリケーション、Webサイト、その他のデジタル製品のユーザーインターフェイスを向上させるために使用されています。

### ライセンス

Material Designアイコンは、Apache Licenseバージョン2.0のライセンスの下で配布されています。ライセンスのコピーは次のURLから入手できます。

<http://www.apache.org/licenses/LICENSE-2.0>

適用法で要求されるか、書面で同意されていない限り、ライセンスに基づいて配布されるソフトウェアは、「現状のまま」の状態で保証や条件なしに配布され、明示的または黙示的に関係ありません。ライセンスの特定の言語により、ライセンスに基づく許可と制限が規定されています。

### 確認

Googleがこれらのアイコンをコミュニティに公開し、開発者やデザイナーがプロジェクトの視覚的魅力とユーザーエクスペリエンスを向上させることができることに対し、謝意を表したいと思います。


## simple_planning_simulator

## 目的 / ユースケース

このノードは、シンプルな車両モデルを使用して、2Dの車両コマンドにおける車両の運動をシミュレートします。

## 設計

このシミュレータの目的は、PlanningモジュールおよびControlモジュールの統合テストを行うことです。センシングや認識はシミュレートせず、純粋なC++だけで実装され、GPUを使用しません。

## 仮定 / 制限事項

- 2Dの運動のみをシミュレートします。
- 衝突やセンシングなどの物理的な操作は実行せず、車両ダイナミクスの積分結果のみを計算します。

## 入出力 / API

### 入出力

- **入力/initialpose** [`geometry_msgs/msg/PoseWithCovarianceStamped`]: 初期姿勢
- **入力/ackermann_control_command** [`autoware_control_msgs/msg/Control`]: 車両を誘導するターゲットコマンド
- **入力/manual_ackermann_control_command** [`autoware_control_msgs/msg/Control`]: 車両を誘導する手動ターゲットコマンド（control_mode_request = Manualの場合に使用）
- **入力/gear_command** [`autoware_vehicle_msgs/msg/GearCommand`]: ターゲットのギアコマンド
- **入力/manual_gear_command** [`autoware_vehicle_msgs/msg/GearCommand`]: ターゲットのギアコマンド（control_mode_request = Manualの場合に使用）
- **入力/turn_indicators_command** [`autoware_vehicle_msgs/msg/TurnIndicatorsCommand`]: ターゲットのターンインジケーターコマンド
- **入力/hazard_lights_command** [`autoware_vehicle_msgs/msg/HazardLightsCommand`]: ターゲットのハザードライトコマンド
- **入力/control_mode_request** [`tier4_vehicle_msgs::srv::ControlModeRequest`]: 自動運転/手動運転のモード変更

### 出力

- `/tf` [`tf2_msgs/msg/TFMessage`]: シミュレートされた車両姿勢（base_link）
- `/output/odometry` [`nav_msgs/msg/Odometry`]: シミュレートされた車両姿勢とツイスト
- `/output/steering` [`autoware_vehicle_msgs/msg/SteeringReport`]: シミュレートされたステアリング角度
- `/output/control_mode_report` [`autoware_vehicle_msgs/msg/ControlModeReport`]: 現在の制御モード（自動運転/手動運転）
- `/output/gear_report` [`autoware_vehicle_msgs/msg/ControlModeReport`]: シミュレートされたギア
- `/output/turn_indicators_report` [`autoware_vehicle_msgs/msg/ControlModeReport`]: シミュレートされたターンインジケーターの状態
- `/output/hazard_lights_report` [`autoware_vehicle_msgs/msg/ControlModeReport`]: シミュレートされたハザードライトの状態

## 内部動作 / アルゴリズム

### コモンパラメーター

| 名称                  | タイプ   | 説明                                                                                                                                              | デフォルト値        |
| :-------------------- | :----- | :-------------------------------------------------------------------------------------------------------------------------------------------------- | :------------------- |
| simulated_frame_id    | 文字列 | 出力tf内のchild_frame_idに設定 | "base_link"          |
| origin_frame_id       | 文字列 | 出力tf内のframe_idに設定               | "odom"               |
| initialize_source     | 文字列 | "ORIGIN"の場合は、初期姿勢が(0, 0, 0)に設定され、"INITIAL_POSE_TOPIC"の場合は、ノードが`input/initialpose`トピックが発行されるまで待機 | "INITIAL_POSE_TOPIC" |
| add_measurement_noise | ブール | trueの場合、シミュレーション結果にガウスノイズが加えられる | true                 |
| pos_noise_stddev      | double | 位置ノイズの標準偏差                                                                                                                              | 0.01                 |
| rpy_noise_stddev      | double | オイラー角ノイズの標準偏差                                                                                                                            | 0.0001               |
| vel_noise_stddev      | double | 速度ノイズの標準偏差                                                                                                                              | 0.0                  |
| angvel_noise_stddev   | double | 角速度ノイズの標準偏差                                                                                                                            | 0.0                  |
| steer_noise_stddev    | double | ステアリング角ノイズの標準偏差                                                                                                                        | 0.0001               |

### 車両モデルのパラメータ

#### `vehicle_model_type` オプション

- `IDEAL_STEER_VEL`
- `IDEAL_STEER_ACC`
- `IDEAL_STEER_ACC_GEARED`
- `DELAY_STEER_VEL`
- `DELAY_STEER_ACC`
- `DELAY_STEER_ACC_GEARED`
- `DELAY_STEER_ACC_GEARED_WO_FALL_GUARD`
- `DELAY_STEER_MAP_ACC_GEARED`: ステアリングおよびアクセラレーションコマンドに対して1次元ダイナミクスと時遅れを適用します。シミュレートされたアクセラレーションは、提供されたアクセラレーションマップを通して変換された値によって決定されます。このモデルは、実際の車両でのアクセラレーション逸脱量を用いた正確なシミュレーションに有益です。
- `LEARNED_STEER_VEL`: 学習されたパイソンモデルを起動します。詳細はこちら [here](../learning_based_vehicle_model) をご覧ください。
- `ACTUATION_CMD`: `ACTUATION_CMD` を受け取るシミュレータモデル。この場合、`raw_vehicle_cmd_converter` も起動されます。

`IDEAL` モデルは命令通りに理想的に移動する一方、`DELAY` モデルは時遅れがある1次モデルに基づいて移動します。`STEER` は、モデルがステアリングコマンドを受け取る意味です。`VEL` は、モデルがターゲットの速度コマンドを受け取り、`ACC` モデルはターゲットのアクセラレーションコマンドを受け取ることを意味します。`GEARED` サフィックスは、モーションがギアコマンドを考慮することを意味します。車両はギアに従って1方向にのみ移動します。

次の表は、どのモデルがどのパラメータに対応するかを示しています。モデル名は省略形で書かれています（例: IDEAL_STEER_VEL = I_ST_V）。

| 名称                       | 型   | 説明                                                                                                  | I_ST_V | I_ST_A | I_ST_A_G | D_ST_V | D_ST_A | D_ST_A_G | D_ST_A_G_WO_FG | D_ST_M_ACC_G | L_S_V | デフォルト値 | 単位    |
| :------------------------- | :----- | :------------------------------------------------------------------------------------------------------ | :----- | :----- | :------- | :----- | :----- | :------- | :------------- | :----------- | :---- | :------------ | :------ |
| acc_time_delay             | double | 加速度入力のデッドタイム                                                                                 | x      | x      | x        | x      | o      | o        | o              | o            | x     | 0.1           | [s]     |
| steer_time_delay           | double | ステアリング入力のデッドタイム                                                                             | x      | x      | x        | o      | o      | o        | o              | o            | x     | 0.24          | [s]     |
| vel_time_delay             | double | 速度入力のデッドタイム                                                                                 | x      | x      | x        | o      | x      | x        | x              | x            | x     | 0.25          | [s]     |
| acc_time_constant          | double | 1次の加速度動特性の時間定数                                                                              | x      | x      | x        | x      | o      | o        | o              | o            | x     | 0.1           | [s]     |
| steer_time_constant        | double | 1次のステアリング動特性の時間定数                                                                          | x      | x      | x        | o      | o      | o        | o              | o            | x     | 0.27          | [s]     |
| steer_dead_band            | double | ステアリング角のデッドバンド                                                                               | x      | x      | x        | o      | o      | o        | o              | x            | x     | 0.0           | [rad]   |
| vel_time_constant          | double | 1次の速度動特性の時間定数                                                                              | x      | x      | x        | o      | x      | x        | x              | x            | x     | 0.5           | [s]     |
| vel_lim                    | double | 速度の上限                                                                                          | x      | x      | x        | o      | o      | o        | o              | o            | x     | 50.0          | [m/s]   |
| vel_rate_lim               | double | 加速度の上限                                                                                          | x      | x      | x        | o      | o      | o        | o              | o            | x     | 7.0           | [m/ss]  |
| steer_lim                  | double | ステアリング角の上限                                                                                    | x      | x      | x        | o      | o      | o        | o              | o            | x     | 1.0           | [rad]   |
| steer_rate_lim             | double | ステアリング角変化率の上限                                                                              | x      | x      | x        | o      | o      | o        | o              | o            | x     | 5.0           | [rad/s] |
| steer_bias                 | double | ステアリング角のバイアス                                                                                | x      | x      | x        | o      | o      | o        | o              | o            | x     | 0.0           | [rad]   |
| debug_acc_scaling_factor   | double | 加速度コマンドのスケーリング係数                                                                          | x      | x      | x        | x      | o      | o        | o              | x            | x     | 1.0           | [-]     |
| debug_steer_scaling_factor | double | ステアリングコマンドのスケーリング係数                                                                          | x      | x      | x        | x      | o      | o        | o              | x            | x     | 1.0           | [-]     |
| acceleration_map_path      | string | 速度と理想的な加速度を実際の加速度に変換するためのCSVファイルのパス                               | x      | x      | x        | x      | x      | x        | x              | o            | x     | -             | [-]     |
| model_module_paths         | string | モデルを実装するPythonモジュールのパス                                                                   | x      | x      | x        | x      | x      | x        | x              | x            | o     | -             | [-]     |
| model_param_paths          | string | モデルパラメータが格納されているファイルのパス（パラメータファイルが必要ない場合は空文字列可） | x      | x      | x        | x      | x      | x        | x              | x            | o     | -             | [-]     |
| model_class_names          | string | モデルを実装するクラスの名前                                                                            | x      | x      | x        | x      | x      | x        | x              | x            | o     | -             | [-]     |

_注意:_ パラメータ`model_module_paths`、`model_param_paths`、および`model_class_names`は同じ長にする必要があります。

`acceleration_map`は`DELAY_STEER_MAP_ACC_GEARED`のみに使用され、垂直軸に加速コマンド、水平軸に自車速度を表示します。各セルはシミュレータの運動計算で実際に使用される変換された加速コマンドを表しています。中間値は線形補間されます。

`acceleration_map.csv`の例


```csv
default,  0.00,  1.39,  2.78,  4.17,  5.56,  6.94,  8.33,  9.72, 11.11, 12.50, 13.89, 15.28, 16.67
-4.0,    -4.40, -4.36, -4.38, -4.12, -4.20, -3.94, -3.98, -3.80, -3.77, -3.76, -3.59, -3.50, -3.40
-3.5,    -4.00, -3.91, -3.85, -3.64, -3.68, -3.55, -3.42, -3.24, -3.25, -3.00, -3.04, -2.93, -2.80
-3.0,    -3.40, -3.37, -3.33, -3.00, -3.00, -2.90, -2.88, -2.65, -2.43, -2.44, -2.43, -2.39, -2.30
-2.5,    -2.80, -2.72, -2.72, -2.62, -2.41, -2.43, -2.26, -2.18, -2.11, -2.03, -1.96, -1.91, -1.85
-2.0,    -2.30, -2.24, -2.12, -2.02, -1.92, -1.81, -1.67, -1.58, -1.51, -1.49, -1.40, -1.35, -1.30
-1.5,    -1.70, -1.61, -1.47, -1.46, -1.40, -1.37, -1.29, -1.24, -1.10, -0.99, -0.83, -0.80, -0.78
-1.0,    -1.30, -1.28, -1.10, -1.09, -1.04, -1.02, -0.98, -0.89, -0.82, -0.61, -0.52, -0.54, -0.56
-0.8,    -0.96, -0.90, -0.82, -0.74, -0.70, -0.65, -0.63, -0.59, -0.55, -0.44, -0.39, -0.39, -0.35
-0.6,    -0.77, -0.71, -0.67, -0.65, -0.58, -0.52, -0.51, -0.50, -0.40, -0.33, -0.30, -0.31, -0.30
-0.4,    -0.45, -0.40, -0.45, -0.44, -0.38, -0.35, -0.31, -0.30, -0.26, -0.30, -0.29, -0.31, -0.25
-0.2,    -0.24, -0.24, -0.25, -0.22, -0.23, -0.25, -0.27, -0.29, -0.24, -0.22, -0.17, -0.18, -0.12
 0.0,     0.00,  0.00, -0.05, -0.05, -0.05, -0.05, -0.08, -0.08, -0.08, -0.08, -0.10, -0.10, -0.10
 0.2,     0.16,  0.12,  0.02,  0.02,  0.00,  0.00, -0.05, -0.05, -0.05, -0.05, -0.08, -0.08, -0.08
 0.4,     0.38,  0.30,  0.22,  0.25,  0.24,  0.23,  0.20,  0.16,  0.16,  0.14,  0.10,  0.05,  0.05
 0.6,     0.52,  0.52,  0.51,  0.49,  0.43,  0.40,  0.35,  0.33,  0.33,  0.33,  0.32,  0.34,  0.34
 0.8,     0.82,  0.81,  0.78,  0.68,  0.63,  0.56,  0.53,  0.48,  0.43,  0.41,  0.37,  0.38,  0.40
 1.0,     1.00,  1.08,  1.01,  0.88,  0.76,  0.69,  0.66,  0.58,  0.54,  0.49,  0.45,  0.40,  0.40
 1.5,     1.52,  1.50,  1.38,  1.26,  1.14,  1.03,  0.91,  0.82,  0.67,  0.61,  0.51,  0.41,  0.41
 2.0,     1.80,  1.80,  1.64,  1.43,  1.25,  1.11,  0.96,  0.81,  0.70,  0.59,  0.51,  0.42,  0.42
```

##### ACTUATION_CMD モデル

通常、simple_planning_simulator は Control コマンドを受信して動作しますが、ACTUATION_CMD モデルを選択すると、Control コマンドの代わりに Actuation コマンドを受信します。このモデルでは、実際の車両に送信される車両コマンドを使用して運動をシミュレートできます。したがって、このモデルを選択すると、raw_vehicle_cmd_converter も立ち上がります。

convert_steer_cmd_method には、「vgr」と「steer_map」の 2 つのオプションがあります。「vgr」（可変ギアレシオ）を選択すると、ステアリングホイール角度がアクチュエーションコマンドとして送信され、その値がモデルを動かすためのステアリングタイヤ角度に変換されるものとみなされます。「steer_map」を選択すると、任意の値がアクチュエーションコマンドとして送信され、その値がモデルを動かすためのステアリングタイヤ速度に変換されるものとみなされます。任意の値というのは、EPS（電子パワーステアリング）電圧のようなものです。enable_pub_steer は、ステアリングタイヤ角度をパブリッシュするかどうかを決定します。false の場合は、アクチュエーションステータスから他のノード（例：raw_vehicle_cmd_converter）に変換されてパブリッシュされると予想されます。

![vgr_sim](./media/vgr_sim.drawio.svg)


```yaml

```

ACTUATION_CMDで使用するパラメータは次のとおりです。

| 名前                     | 型   | 説明                                                                                                                                                              | 単位 |
| :----------------------- | :----- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :--- |
| accel_time_delay         | double | 加速度入力のデッドタイム                                                                                                                                     | [秒]  |
| accel_time_constant      | double | 1 次加速度動特性のタイムコンスタント                                                                                                                     | [秒]  |
| brake_time_delay         | double | ブレーキ入力のデッドタイム                                                                                                                                            | [秒]  |
| brake_time_constant      | double | 1 次ブレーキ動特性のタイムコンスタント                                                                                                                            | [秒]  |
| convert_accel_cmd        | bool   | true の場合、コマンドは加速度作動値に変換されて送信されると想定され、シミュレータ内で加速度値に戻されます。                                      | [-]  |
| convert_brake_cmd        | bool   | true の場合、コマンドはブレーキ作動値に変換されて送信されると想定され、シミュレータ内で加速度値に戻されます。                                      | [-]  |
| convert_steer_cmd        | bool   | true の場合、コマンドは操舵作動値に変換されて送信されると想定され、シミュレータ内で操舵速度値に戻されます。                                    | [-]  |
| convert_steer_cmd_method | bool   | 操舵コマンドを変換する方法。「vgr」と「steer_map」から選択できます。                                                                                     | [-]  |
| vgr_coef_a               | double | 可変ギア比の係数 a の値                                                                                                                                | [-]  |
| vgr_coef_b               | double | 可変ギア比の係数 b の値                                                                                                                                | [-]  |
| vgr_coef_c               | double | 可変ギア比の係数 c の値                                                                                                                                | [-]  |
| enable_pub_steer         | bool   | ステアリングタイヤ角を公開するかどうか。false の場合、actuation_status から他のノード（例: raw_vehicle_cmd_converter）で変換され、公開される必要があります。 | [-]

<!-- deadzone_delta_steer | double | ステアリング・ダイナミクスにおけるデッドゾーン | x | x | x | o | o | 0.0 | [rad] | | -->

_注意_: ステアリング・ダイナミクス/速度・加速度ダイナミクスは、_遅延_モデルにおけるデッドタイムを持った1次システムでモデリングされています。_時定数_の定義は、ステップ応答が最終値の63%に上昇するまでの時間です。_デッドタイム_は、制御入力に対する応答の遅延です。

### LEARNED_STEER_VELモデルの例

`LEARNED_STEER_VEL`の仕組みを示すために、いくつかの基本モデルを作成しました。

1. 基本的なPythonモデルを含む[ライブラリ](https://github.com/atomyks/control_analysis_pipeline/tree/v0.1_autoware)をインストールします (ブランチ: `v0.1_autoware`)

2. `src/vehicle/sample_vehicle_launch/sample_vehicle_description/config/simulator_model.param.yaml`ファイルの`vehicle_model_type`を`LEARNED_STEER_VEL`に設定します。同じファイルで、次のパラメータを設定します。これらのモデルはテスト用であり、パラメータファイルは必要ありません。


```yaml
model_module_paths:
  [
    "control_analysis_pipeline.autoware_models.vehicle.kinematic",
    "control_analysis_pipeline.autoware_models.steering.steer_example",
    "control_analysis_pipeline.autoware_models.drive.drive_example",
  ]
model_param_paths: ["", "", ""]
model_class_names: ["KinematicModel", "SteerExample", "DriveExample"]
```

### デフォルトの TF 設定

車両が `odom`->`base_link` tf を出力するため、このシミュレータは同フレーム ID 設定の tf を出力します。
simple_planning_simulator.launch.py では、通常はローカリゼーションモジュール (例: NDT) で推定される `map`->`odom` tf を出力するノードも起動されます。このシミュレータモジュールによって出力される tf は理想的な値であるため、`odom`->`map` は常に 0 になります。

### (注意点) ピッチ角計算

車両のピッチ角は次の方法で計算されます。

![ピッチ角計算](./media/pitch-calculation.drawio.svg)

注意: 画像の最下行に示されているようにライン方向に対して運転することはサポートされておらず、説明目的のみを示しています。

## エラー検出と処理

入力の検証は車両モデルタイプの有効性テストのみです。

## セキュリティに関する考慮事項

<!-- 必要 -->
<!-- 考慮事項:
 - なりすまし (偽の入力をどのようにチェックして処理しますか?)
 - 改ざん (改ざんされた入力をどのようにチェックして処理しますか?)
 - 否認 (外部の行動者はあなたにどのように影響を与えますか?)
 - 情報漏えい (データは漏洩しますか?)
 - サービス拒否 (スパミングをどのように処理しますか?)
 - 権限昇格 (実行中に権限レベルを変更する必要がありますか?) -->

## 参照/外部リンク

Autoware.AI で最初に開発されました。以下のリンクを参照してください。

<https://github.com/Autoware-AI/simulation/tree/master/wf_simulator>

## 今後の拡張/未実装の部分

 - 車両モデルの精度の向上 (例: ステアリングデッドゾーンとスリップ挙動の追加)
 - 擬似点群または擬似認識結果を出力するモジュールとの連携


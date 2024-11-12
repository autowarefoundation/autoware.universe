## Planning Validator

`autoware_planning_validator` は、パブリッシュされる前に軌道が有効であることを確認するモジュールです。有効化のステータスは `/diagnostics` と `/validation_status` トピックで確認できます。無効な軌道が検出された場合、`autoware_planning_validator` は選択されたオプションに従って軌道を処理します: "0. 軌道そのままパブリッシュ", "1. 軌道のパブリッシュを停止", "2. 最後に検証された軌道をパブリッシュ".

![autoware_planning_validator](./image/planning_validator.drawio.svg)

## サポートしている機能

軌道検証でサポートしている機能は次の通りで、パラメータでしきい値を設定できます。

- **無効なフィールド** : 例: 無限大、NaN
- **軌道ポイント間隔** : 軌道ポイントの間隔が大きすぎる場合無効
- **曲率** : 与えられた車両の運動特性上実行不可能なほど軌道に急カーブがある場合無効
- **相対角度** : 軌道ポイントのシーケンスでヨー角が急激に変化した場合無効
- **側方加速度** : 予想される側方加速度/減速度が大きすぎる場合無効
- **縦方向加速度/減速度** : 軌道ポイントの加速度/減速度が大きすぎる場合無効
- **ステアリング角度** : 軌道曲率から推定される予想ステアリング値が大きすぎる場合無効
- **ステアリング角速度** : 予想ステアリング角速度値が大きすぎる場合無効
- **速度偏差** : 計画速度が自車速度からかけ離れている場合無効
- **距離偏差** : 自車が軌道から離れすぎている場合無効
- **縦方向距離偏差** : 軌道が自車から縦方向に離れすぎている場合無効
- **前方軌道長** : 与えられた減速度内で停止するため軌道長が十分でない場合無効

次の機能は実装される予定です。

- **(TODO) TTC 計算** : 軌道上の予想時間交通距離が短すぎる場合無効

## 出力/入出力

### 出力

`autoware_planning_validator` は次の出力を出力します:
- **/diagnostics** : このモジュールの診断ステータスに関する情報
- **/validation_status** : 軌道の検証ステータス

### 入出力

`autoware_planning_validator` は次の入力を取ります:

| 名称                 | タイプ                              | 説明                                    |
| -------------------- | --------------------------------- | ---------------------------------------------- |
| `~/input/kinematics` | nav_msgs/Odometry                 | 自車位置と速度                             |
| `~/input/trajectory` | autoware_planning_msgs/Trajectory | 本ノードで検証するターゲット軌跡 |

### 出力

次のものを出力します。

| 名称                         | タイプ                                       | 説明                                                                      |
| ---------------------------- | ------------------------------------------ | ------------------------------------------------------------------------- |
| `~/output/trajectory`       | autoware_planning_msgs/Trajectory           | 検証済みの走行軌跡                                                     |
| `~/output/validation_status` | planning_validator/PlanningValidatorStatus | バリデータのステータスで、走行軌跡が有効/無効の理由を通知します |
| `/diagnostics`              | diagnostic_msgs/DiagnosticStatus            | エラーを報告する診断                                                     |

## パラメータ

`autoware_planning_validator` には、以下のパラメータを設定できます。

### システムパラメータ

| 名前 | タイプ | 説明 | デフォルト値 |
|---|---|---|---|
| `invalid_trajectory_handling_type` | int | 無効な経路が見つかった場合の処理を設定します。 <br>0: 無効でも経路を公開する <br>1: 経路の公開を停止する <br>2: 最後に検証済の経路を公開する。 | 0 |
| `publish_diag` | bool | 連続した無効な経路の数がこの閾値を超えると、DiagがERRORに設定されます。(例: threshold = 1の場合、たとえ経路が無効でも、次の経路が有効であればDiagはERRORになりません。) | true |
| `diag_error_count_threshold` | int | trueの場合、診断メッセージが公開されます。 | true |
| `display_on_terminal` | bool | エラーメッセージをターミナルに表示する | true |

### アルゴリズムパラメータ

#### スレッショルド

インデックスが以下の閾値を超えた場合、入力軌跡は無効として検出されます。

| 名前 | タイプ | 説明 | デフォルト値 |
|---|---|---|---|
| `thresholds.interval` | 数値 | 2つの近隣の経路ポイント間の距離の無効しきい値 [`m`] | 100.0 |
| `thresholds.relative_angle` | 数値 | 2つの近隣の経路ポイント間の相対角度の無効しきい値 [`rad`] | 2.0 |
| `thresholds.curvature` | 数値 | 各経路ポイントの曲率の無効しきい値 [`1/m`] | 1.0 |
| `thresholds.lateral_acc` | 数値 | 各経路ポイントの横加速度の無効しきい値 [`m/ss`] | 9.8 |
| `thresholds.longitudinal_max_acc` | 数値 | 各経路ポイントの最大縦加速度の無効しきい値 [`m/ss`] | 9.8 |
| `thresholds.longitudinal_min_acc` | 数値 | 各経路ポイントの最小縦減速度の無効しきい値 [`m/ss`] | -9.8 |
| `thresholds.steering` | 数値 | 各経路ポイントの操舵角の無効しきい値 [`rad`] | 1.414 |
| `thresholds.steering_rate` | 数値 | 各経路ポイントの操舵角速度の無効しきい値 [`rad/s`] | 10.0 |
| `thresholds.velocity_deviation` | 数値 | エゴの速度とエゴに最も近い経路ポイント間の速度偏差の無効しきい値 [`m/s`] | 100.0 |
| `thresholds.distance_deviation` | 数値 | エゴの位置とエゴに最も近い経路ポイント間の距離偏差の無効しきい値 [`m`] | 100.0 |
| `parameters.longitudinal_distance_deviation` | 数値 | エゴの位置と経路間の縦方向距離偏差の無効しきい値 [`m`] | 2.0 |

#### パラメータ

しきい値の計算などに使用されるパラメータ

| `parameters.forward_trajectory_length_acceleration` | double | この値は、必要な軌道長の算出に使用されます。 | -5.0 |
| `parameters.forward_trajectory_length_margin` | double | 自車が軌道の終点をわずかに過ぎてもエラーが発生しないようにするための、必要な軌道長の余白。 | 2.0 |


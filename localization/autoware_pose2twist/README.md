## autoware_pose2twist

## 目的

この `autoware_pose2twist` は入力された履歴ポーズから速度を計算します。このノードは計算されたツイストに加えて、デバッグを簡素化するために浮動小数点メッセージとして直線 x 成分と角度 z 成分を出力します。

`twist.linear.x` は `sqrt(dx * dx + dy * dy + dz * dz) / dt` で計算され、`y` フィールドと `z` フィールドの値は 0 になります。
`twist.angular` は各フィールドについて `relative_rotation_vector / dt` で計算されます。

## 入出力

### 入力
- `/diff_poses`: 車両の過去の `post resampling` ポーズ順序（ジオリファレンス済み）
- `/current_pose`: 車両の自車位置と姿勢（ジオリファレンス済み）

### 出力
- `/twist`: 速度
- `/linear_x`: `twist.linear.x` の値
- `/linear_y`: `twist.linear.y` の値
- `/linear_z`: `twist.linear.z` の値
- `/angular_x`: `twist.angular.x` の値
- `/angular_y`: `twist.angular.y` の値
- `/angular_z`: `twist.angular.z` の値
- `/velocity_error`: 速度エラー
- `/velocity_error_tran`: 平行移動速度逸脱量
- `/velocity_error_rot`: 回転速度逸脱量
- `/acceleration_error`: 加速度エラー
- `/acceleration_error_tran`: 平行移動加速度逸脱量
- `/acceleration_error_rot`: 回転加速度逸脱量
- `/filtered_velocity`: フィルタリングされた速度

| 名称 | 種別 | 説明 |
|---|---|---|
| pose | geometry_msgs::msg::PoseStamped | 速度計算に使用する姿勢のソース |

## 自動運転ソフトウェアドキュメント

### Planningモジュール

#### VehicleBehaviorPrediction（車両挙動予測）

VehicleBehaviorPredictionコンポーネントは、他の車両の挙動を予測します。この情報は、Planningコンポーネントによって、自車位置の決定と回避経路の生成に使用されます。

VehicleBehaviorPredictionコンポーネントは、入力として、以下のデータを使用します。

- LiDARとレーダーからの検出結果
- センサーの不確実性
- 道路ネットワーク情報

VehicleBehaviorPredictionコンポーネントは、以下のアルゴリズムを使用して、他の車両の挙動を予測します。

- Kalmanフィルター
- 粒子フィルター
- 深層学習

#### Planning（経路計画）

Planningコンポーネントは、自車位置を決定し、回避経路を生成します。この情報は、Controlコンポーネントによって、車両の制御に使用されます。

Planningコンポーネントは、入力として、以下のデータを使用します。

- VehicleBehaviorPredictionコンポーネントからの予測車両挙動
- 自車位置
- 道路ネットワーク情報

Planningコンポーネントは、以下のアルゴリズムを使用して、自車位置を決定し、回避経路を生成します。

- 動的計画法
- グラフ探索
- 最適化

### Controlモジュール

#### LongitudinalControl（縦方向制御）

LongitudinalControlコンポーネントは、車両の縦方向の動き（速度と加速度）を制御します。この情報は、ブレーキ、アクセル、ギアの制御に使用されます。

LongitudinalControlコンポーネントは、入力として、以下のデータを使用します。

- Planningコンポーネントからの自車位置
- 車両の速度と加速度
- 障害物検出結果

LongitudinalControlコンポーネントは、以下のアルゴリズムを使用して、車両の縦方向の動きを制御します。

- PID制御
- モデル予測制御

#### LateralControl（横方向制御）

LateralControlコンポーネントは、車両の横方向の動き（操舵角）を制御します。この情報は、ステアリングシステムの制御に使用されます。

LateralControlコンポーネントは、入力として、以下のデータを使用します。

- Planningコンポーネントからの自車位置
- 車両の速度と加速度
- 障害物検出結果

LateralControlコンポーネントは、以下のアルゴリズムを使用して、車両の横方向の動きを制御します。

- PID制御
- モデル予測制御

### システムインテグレーション

Autowareシステムは、以下のコンポーネントで構成されています。

- Planningモジュール
- Controlモジュール
- センサーインターフェース
- システムモニター

これらのコンポーネントは、CANバスを介して相互に通信します。

### 評価

Autowareシステムは、以下の指標を使用して評価されます。

- 到達率
- 衝突回数
- `post resampling`経路逸脱量
- 加速度逸脱量
- 速度逸脱量

| 名前      | 種類                                  | 説明                                      |
| --------- | ------------------------------------- | --------------------------------------------- |
| twist     | geometry_msgs::msg::TwistStamped      | 入力された姿勢履歴から計算した捻り率。 |
| linear_x  | tier4_debug_msgs::msg::Float32Stamped | 出力捻り率の線形 x フィールド。            |
| angular_z | tier4_debug_msgs::msg::Float32Stamped | 出力捻り率の角速度 z フィールド。          |

## パラメータ

なし

## 仮定 / 既知の制約

なし


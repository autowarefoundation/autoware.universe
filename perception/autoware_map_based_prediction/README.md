## map_based_prediction

## 機能

`map_based_prediction`モジュールは、地図の形状と周囲環境に応じて、他の車両や歩行者の未来の経路（とその確率）を予測します。

## 仮定

- 標的障礙物に関する以下の情報が必要
  - ラベル（人物、自動車などのタイプ）
  - 現在の時間でのオブジェクトの位置と、将来の時間の予測位置
- 周囲環境に関する以下の情報が必要
  - Lanelet2形式の道路網情報

## 内部動作 / アルゴリズム

### フローチャート

<div align="center">
  <img src="media/map_based_prediction_flow.drawio.svg" width=20%>
</div>

### 道路利用者の経路予測

#### 古いオブジェクト履歴の削除

オブジェクトの位置、速度、時刻などの情報を格納したオブジェクトのタイムシリーズデータを格納して、車両の経路を決定し、数時間の車線変更を検出します。

#### 現在のレーンの取得とオブジェクト履歴の更新

各標的オブジェクトに対して、以下の条件を満たす1つ以上のレーンレットを検索し、ObjectDataに格納します。

- オブジェクトの重心はレーンレット内に存在する必要があります。
- レーンレットの中心線には2つ以上の点が必要です。
- レーンレットとオブジェクトの方向の角度差は、パラメータで指定されたしきい値以内である必要があります。
  - 角度の反転は許可され、条件は`diff_yaw < threshold or diff_yaw > pi - threshold`です。
- レーンレットは、過去の履歴に記録されたレーンレットから到達可能である必要があります。

#### 予測された基準経路の取得

- 基準経路の取得：
  - 関連付けられたレーンレットからオブジェクトの基準経路を作成します。
- オブジェクト機動の予測：
  - オブジェクトの予測経路を生成します。
  - オブジェクトの履歴と最初のステップで取得した基準経路に基づいて、`Lane Follow`、`Left Lane Change`、`Right Lane Change`のそれぞれの機動に確率を割り当てます。
  - 車線変更の決定は、2つのドメインに基づいています。
    - 幾何学的ドメイン：オブジェクトの重心とレーンの左/右境界との横方向距離
    - 時間ドメイン：オブジェクトが左/右境界に到達するまでの推定時間マージン

左車線変更検出の条件は次のとおりです。

- 左車線境界までの距離が右車線境界までの距離よりも小さいか確認します。
- 左車線境界までの距離が`dist_threshold_to_bound_`よりも小さいか確認します。
- 横方向速度方向が左車線境界に向かっていないか確認します。
- 左車線境界に到達するまでの時間が`time_threshold_to_bound_`よりも小さいか確認します。

車線変更のロジックは、以下の図に示されています。パラメータの調整方法の例は、後で説明します。

![車線変更検出](./media/lane_change_detection.drawio.svg)

- **オブジェクト確率の計算:**
  - 上記で得られた経路確率は、オブジェクトの自車位置と角度に基づいて計算されます。
- **スムーズな動作のための予測経路の微調整:**
  - 生成された予測経路は、車両のダイナミクスを考慮するために再計算されます。
  - 経路は、横方向/縦方向の運動に対して 4 次/5 次スプラインで実装された最小ジャーク軌道で計算されます。

### 車線変更検出ロジックの調整

現在、車線変更検出を調整するためのパラメータは 3 つあります。

- `dist_threshold_to_bound_`: 車線変更車両が許容される車線境界からの最大距離
- `time_threshold_to_bound_`: 車線変更車両が境界に到達できる最大時間
- `cutoff_freq_of_velocity_lpf_`: 横方向速度のローパスフィルタのカットオフ周波数

これらのパラメータは、以下の表に示すように ROS パラメータで変更できます。

| 設定名                                              | デフォルト値 |
| ---------------------------------------------------- | ------------- |
| `レーン変更検出の距離閾値`                          | `1.0` [m]     |
| `レーン変更検出の時間閾値`                          | `5.0` [s]     |
| `レーン変更検出用速度のカットオフ周波数`             | `0.1` [Hz]    |

#### しきい値パラメータの調整

次の 2 つのパラメータを増やすと、車線変更推定の速度が低下し、安定します。

通常は `time_threshold_for_lane_change_detection` のみを調整することを推奨します。車線変更の判断において、より重要な要素だからです。

#### 横方向速度の計算の調整

横方向速度の計算も車線変更判断において非常に重要な要素です。時間領域判断で使用されるからです。

車線境界線に達する予測時間は、次の式で計算されます。

$$
t_{predicted} = \dfrac{d_{lat}}{v_{lat}}
$$

ここで $d_{lat}$ と $v_{lat}$ は、それぞれ車線境界線までの横方向距離と横方向速度を表します。

横方向速度に対するローパスフィルタのカットオフ周波数を下げると、車線変更判断が安定しますが遅くなります。設定は非常に慎重ですが、車線変更判断を早くしたい場合はこのパラメータを大きくできます。

参考までに、横方向速度の計算方法を示します。

| 横速度計算手法                           | 式                           | 説明                                                                                                                                                                                                                               |
| ------------------------------------------------------------- | ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [**適用**済み] 横距離のタイムデリバティブ             | $\dfrac{\Delta d_{lat}}{\Delta t}$ | 現在、この手法は曲がりくねった道路に対処するために使用されています。この時間微分は容易にノイズを生じるため、低域通過フィルターも使用して速度を滑らかにしています。                                                      |
| [適用なし] オブジェクトの速度の横方向への投影 | $v_{obj} \sin(\theta)$             | 通常、オブジェクトの速度は横距離のタイムデリバティブよりもノイズが少なくなります。ただし、車線とオブジェクトの方向のヨー角差 $\theta$ が時々不連続になるため、この手法を採用しませんでした。 |

現在、ローパスフィルターを使用した高階メソッドで横速度を算出しています。

### パス生成

パス生成はフレネフレームで生成されます。パスは、次の手順で生成されます。

1. 基準パスのフレネフレームを取得します。
2. オブジェクトの自車位置とオブジェクトの未来位置のフレネフレームを生成します。
3. フレネフレームの各縦横座標でパスを最適化します。(開始条件と終了条件に五次多項式を当てはめます。)
4. パスをグローバル座標に変換します。

詳細については、論文 [2] を参照してください。

#### 横方向パス形状の調整

`lateral_control_time_horizon` パラメータは、横方向パス形状の調整に使用します。このパラメータは、基準パスに到達する時間を計算するために使用されます。値が小さいほど、基準パスに素早く到達するようにパスが生成されます。(ほとんどは車線の真ん中です。)

#### 横加速度制約を使用して予測パスを刈り取る(車両障害物に対して)

生成された車両パスに最大横加速度制約を適用することができます。このチェックは、カーブを走行するときに車両が横加速度のしきい値 `max_lateral_accel` を超えずに予測パスを実行できるかどうかを確認します。それが不可能な場合は、車両が減速時に適宜 `min_acceleration_before_curve` の減速度でカーブを走行して制約を満たすことができるかどうかを確認します。それでも不可能な場合は、パスは削除されます。

現在、横加速度制約を調整するためのパラメータを3つ用意しています。

- `check_lateral_acceleration_constraints_`: 制約チェックを有効にします。
- `max_lateral_accel_`: 予測パスに対して許容される最大横加速度(絶対値)。
- `min_acceleration_before_curve_`: 車両がカーブに到達する前に理論的に使用される最小加速度(負にする必要があります)。

Rosparam で以下の表に示すパラメータを変更できます。

| パラメータ名                              | デフォルト値 |
| ---------------------------------------- | ------------ |
| `check_lateral_acceleration_constraints` | `false` [bool] |
| `max_lateral_accel`                      | `2.0` [m/s^2] |
| `min_acceleration_before_curve`          | `-2.0` [m/s^2] |

## 自動車加速度を使用した経路予測（障害車両用）

デフォルトでは、`map_based_prediction`モジュールは現在の障害物の速度を使用して予測経路長を計算します。ただし、障害物の現在の加速度を使用して予測経路長を計算することもできます。

### 減衰加速度モデル

このモジュールは車両の経路を検出後数秒先まで予測しようとするため、現在の車両の加速度を一定とみなすことは現実的ではありません（車両が検出後`prediction_time_horizon`秒間加速し続けるとは想定されていません）。代わりに、減衰加速度モデルが使用されます。減衰加速度モデルでは、車両の加速度は次のようにモデル化されます。

$\ a(t) = a\_{t0} \cdot e^{-\lambda \cdot t} $

ここで$\ a\_{t0} $は検出時の車両加速度で、$\ \lambda $は減衰定数$\ \lambda = \ln(2) / hl $、$\ hl $は指数関数半減期です。

さらに、時間の経過に対する$\ a(t) $の積分により、速度$\ v(t) $および距離$\ x(t) $の式が得られます。

$\ v(t) = v*{t0} + a*{t0} \* (1/\lambda) \cdot (1 - e^{-\lambda \cdot t}) $

および

$\ x(t) = x*{t0} + (v*{t0} + a*{t0} \* (1/\lambda)) \cdot t + a*{t0}(1/λ^2)(e^{-\lambda \cdot t} - 1) $

このモデルでは、予測経路長に対する障害物が検出された瞬間の加速度の影響は減少しますが、それでも考慮されます。この機能はまた、障害物が道路の制限速度（調整可能な係数で乗算）を超えて加速しない可能性も考慮します。

現在、経路予測における障害物加速度の使用を調整するためのパラメータを3つ提供しています。

- `use_vehicle_acceleration`: この機能を有効にします。
- `acceleration_exponential_half_life`: 減衰加速度モデルは、現在の車両加速度がこの時間後には半分になると想定します。
- `speed_limit_multiplier`: 障害車両 प्रकारの最大予測速度を、そのレーンレットの法定速度制限にこの値を乗算したものに設定します。この値は1.0以上にする必要があります。

次の表の`rosparam`でこれらのパラメータを変更できます。

| パラメータ名                           | デフォルト値  |
| ------------------------------------ | -------------- |
| `use_vehicle_acceleration`           | `false` [bool] |
| `acceleration_exponential_half_life` | `2.5` [s]      |
| `speed_limit_multiplier`             | `1.5` []       |

### 横断歩道利用者の経路予測

このモジュールは、**歩行者**と**自転車**を横断歩道を使用するオブジェクトとして扱い、オブジェクトが横断歩道を通過する意図を持っていると仮定し、マップと推定したオブジェクトの速度に基づいて予測パスを出力します。オブジェクトが少なくとも次のいずれかの条件を満たす場合です。

- 横断歩道に向かって移動する
- 横断歩道の近くで停止する

<div align="center">
  <img src="images/target_objects.svg" width=90%>
</div>

`prediction_time_horizon` 内に到達可能な横断歩道エントリ ポイントがあり、オブジェクトが上記の条件を満たしている場合、このモジュールは横断歩道エントリ ポイントを経由して反対側を通過する追加の予測パスを出力します。

このモジュールは、対応する信号情報も考慮に入れます。
赤信号が表示されている場合、対象オブジェクトは横断しないと仮定します。
さらに、対象オブジェクトが青信号に対して停止 (移動していない) している場合、対象オブジェクトは横断しないと仮定します。この予測は、信号が青で、オブジェクトが横断する意図がある場合、オブジェクトは移動するはずだという仮定に基づいています。

<div align="center">
  <img src="images/outside_road.svg" width=90%>
</div>

対象オブジェクトが道路または横断歩道内にある場合、このモジュールは横断歩道の出口点に到達するための 1 つまたは 2 つの追加予測パスを出力します。予測パスの数は、オブジェクトが移動しているかどうかに依存します。オブジェクトが移動している場合、このモジュールはオブジェクトの移動方向に存在する出口点に向けた 1 つの予測パスを出力します。一方、オブジェクトが停止している場合、オブジェクトがどちらの出口点に行こうとしているかを推測することは不可能であるため、このモジュールは両側の出口点に向けた 2 つの予測パスを出力します。

<div align="center">
  <img src="images/inside_road.svg" width=90%>
</div>

## 入出力

### 入力

| 名称                                                    | 型                                                    | 説明                                                  |
| ------------------------------------------------------- | ------------------------------------------------------- | -------------------------------------------------------- |
| `~/perception/object_recognition/tracking/objects`      | `autoware_perception_msgs::msg::TrackedObjects`         | 予測パスを持たない追跡オブジェクト。                  |
| `~/vector_map`                                          | `autoware_map_msgs::msg::LaneletMapBin`                 | Lanelet2 Mapのバイナリデータ。                          |
| `~/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 対応する信号機の再配列された情報。                      |

### 自動運転ソフトウェアに関するドキュメント

#### 目次

- [はじめに](#はじめに)
- [Planning（計画）](#planning)
  - [Planning Framework（計画フレームワーク）](#planning-framework)
  - [Central Planner（セントラルプランナー）](#central-planner)
- [Control（制御）](#control)
- [Perception（認知）](#perception)
  - [Object Recognition（物体認識）](#object-recognition)
  - [Detection（検出）](#detection)
- [Simulation（シミュレーション）](#simulation)
  - [Simulation Test Framework（シミュレーションテストフレームワーク）](#simulation-test-framework)
- [評価](#評価)
  - [Metrics and Benchmarks（メトリクスとベンチマーク）](#metrics-and-benchmarks)
- [Deployment（展開）](#deployment)
  - [Autoware Deployment Process（Autoware展開プロセス）](#autoware-deployment-process)
  - [Continuous Integration and Continuous Delivery（継続的インテグレーションと継続的デリバリー）](#continuous-integration-and-continuous-delivery)
- [メンテナンス](#メンテナンス)
  - [Updating Autoware（Autowareのアップデート）](#updating-autoware)
  - [Troubleshooting（トラブルシューティング）](#troubleshooting)

#### はじめに

このドキュメントは、Autowareの自動運転ソフトウェアに関する包括的なガイダンスを提供します。このソフトウェアは、自動運転車両の開発と展開のためのオープンソースのプラットフォームです。このドキュメントでは、ソフトウェアのアーキテクチャ、コンポーネント、デプロイメントプロセスについて説明します。

#### Planning（計画）

**Planning Framework（計画フレームワーク）**

Planning Frameworkは、自動運転車両の経路計画を担当します。障害物回避、車線維持、信号対応などのタスクを処理します。

**Central Planner（セントラルプランナー）**

Central Plannerは、Planning Frameworkの中核コンポーネントです。車両の周囲環境を認識し、安全で効率的な経路を生成します。

#### Control（制御）

Controlコンポーネントは、Planningコンポーネントによって生成された経路に従って車両を制御します。エンジン、ブレーキ、ステアリングなどのアクチュエータを操作します。

#### Perception（認知）

**Object Recognition（物体認識）**

Object Recognitionコンポーネントは、車両の周囲環境にある物体、歩行者、車両を認識します。カメラとLiDARセンサーのデータを処理します。

**Detection（検出）**

Detectionコンポーネントは、認識された物体の位置と速度を推定します。時系列データと確率的推定手法を使用して、堅牢かつ正確な検出を行います。

#### Simulation（シミュレーション）

**Simulation Test Framework（シミュレーションテストフレームワーク）**

Simulation Test Frameworkは、自動運転システムの開発とテストのためのシミュレーション環境を提供します。現実的なシナリオを作成することで、安全性と信頼性を確保できます。

#### 評価

**Metrics and Benchmarks（メトリクスとベンチマーク）**

評価コンポーネントは、自動運転システムのパフォーマンスを評価するためのメトリクスとベンチマークを提供します。安全度、効率性、快適性などの指標を測定します。

#### Deployment（展開）

**Autoware Deployment Process（Autoware展開プロセス）**

Autoware Deployment Processは、Autowareシステムを実際の車両に展開するための段階的なガイドラインを提供します。ソフトウェアのインストール、設定、統合をカバーします。

**Continuous Integration and Continuous Delivery（継続的インテグレーションと継続的デリバリー）**

継続的インテグレーションと継続的デリバリー（CI/CD）のプロセスを使用して、新機能や更新を自動的にビルド、テスト、展開します。これにより、展開プロセスの効率と信頼性が向上します。

#### メンテナンス

**Updating Autoware（Autowareのアップデート）**

Autowareのアップデートにより、最新の機能、バグ修正、セキュリティパッチにアクセスできます。新しいリリースのダウンロードとインストールの手順を提供します。

**Troubleshooting（トラブルシューティング）**

Troubleshootingガイドには、一般的な問題の解決手順が記載されています。診断、ログ分析、デバッグの手順を提供します。

| 名称                         | 型                                              | 説明                                                                                 |
| ---------------------------- | ------------------------------------------------- | --------------------------------------------------------------------------------------- |
| `~/input/objects`            | `autoware_perception_msgs::msg::TrackedObjects`   | 検知物体。デフォルトは `/perception/object_recognition/tracking/objects` に設定されている |
| `~/output/objects`           | `autoware_perception_msgs::msg::PredictedObjects` | 予測経路付き検知物体                                                                 |
| `~/objects_path_markers`     | `visualization_msgs::msg::MarkerArray`            | 可視化用のマーカー                                                                   |
| `~/debug/processing_time_ms` | `std_msgs::msg::Float64`                          | このモジュールの処理時間                                                             |
| `~/debug/cyclic_time_ms`     | `std_msgs::msg::Float64`                          | このモジュールの周期時間                                                               |

## パラメータ

| パラメータ | 単位 | タイプ | 説明 |
|---|---|---|---|
| `enable_delay_compensation` | [-] | bool | 物体の位置に対する時間遅延補正を有効にするためのフラグ |
| `prediction_time_horizon` | [秒] | double | 予測 경로의 예측 시간 지속 기간 |
| `lateral_control_time_horizon` | [秒] | double | 예측 경로가 기준 경로(대부분 차선 중심)에 도달하는 시간 지속 기간 |
| `prediction_sampling_delta_time` | [秒] | double | 예측 경로에서 포인트의 샘플링 시간 |
| `min_velocity_for_map_based_prediction` | [m/s] | double | 이 값보다 높은 속도의 물체에 지도 기반 예측 적용 |
| `min_crosswalk_user_velocity` | [m/s] | double | 보행자의 속도를 계산할 때 사용되는 최소 속도 |
| `max_crosswalk_user_delta_yaw_threshold_for_lanelet` | [라디안] | double | 보행자와 차로 간의 최대 요우 각도 차이를 보행자 경로 예측에 사용 |
| `dist_threshold_for_searching_lanelet` | [m] | double | 물체가 속하는 차로를 검색하는 데 사용되는 각도 임계값 |
| `delta_yaw_threshold_for_searching_lanelet` | [라디안] | double | 물체가 속하는 차로를 검색하는 데 사용되는 각도 임계값 |
| `sigma_lateral_offset` | [m] | double | 물체의 측면 위치에 대한 표준 편차 |
| `sigma_yaw_angle_deg` | [도] | double | 물체의 요우 각도에 대한 표준 편차 |
| `object_buffer_time_length` | [초] | double | 물체의 정보를 저장하는 객체 이력의 시간 범위 |
| `history_time_length` | [초] | double | 예측에 사용되는 객체 정보의 시간 범위 |
| `prediction_time_horizon_rate_for_validate_shoulder_lane_length` | [-] | double | 예측 경로 길이가 차선 길이를 초과하면 예측 경로 비활성화. 이 매개변수는 예측 경로 길이를 제어 |

## 前提条件/既知の制限

- 乗用車、バス、トラックのオブジェクトタイプについて
  - オブジェクトの予測経路は道路構造に従います。
  - オブジェクトが道路上にない場合、予測経路は直線予測によって生成されます。
  - レーンレット上にいるが、道路と異なる方向に移動しているオブジェクトの場合、予測経路は単純に直線です。
  - 予測経路で車両のダイナミクスが適切に考慮されない可能性があります。
- 人とオートバイのオブジェクトタイプについて
  - 予測経路は「横断歩道付近」以外のすべての状況で、単純な直線で生成されます。
- すべての障害物について
  - 加速度情報の不足により、予測では車両の運動が等速と想定されています。

## 参考資料

1. M. Werling, J. Ziegler, S. Kammel, and S. Thrun, “Optimal trajectory generation for dynamic street scenario in a frenet frame,” IEEE International Conference on Robotics and Automation, Anchorage, Alaska, USA, May 2010.
2. A. Houenou, P. Bonnifait, V. Cherfaoui, and Wen Yao, “Vehicle trajectory prediction based on motion model and maneuver recognition,” in 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, nov 2013, pp. 4363-4369.


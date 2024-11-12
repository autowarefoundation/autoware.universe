## コントロールパフォーマンス分析

## 目的

`control_performance_analysis`は、制御モジュールの追従性能を分析し、車両の走行状況を監視するためのパッケージです。

このパッケージは、制御モジュールの結果を定量化するためのツールとして使用されます。
そのため、自動運転のコアロジックに干渉しません。

プランニング、制御、車両からのさまざまな入力に基づいて、このパッケージで定義された`control_performance_analysis::msg::ErrorStamped`として分析の結果を公開します。

`ErrorStamped`メッセージのすべての結果は、曲線のフレネフレームで計算されます。エラーと速度エラーは、次の論文を使用して計算されます。

<!-- cspell: ignore Werling Moritz Groell Lutz Bretthauer Georg -->

`Werling, Moritz & Groell, Lutz & Bretthauer, Georg. (2010). Invariant Trajectory Tracking With a Full-Size Autonomous Road Vehicle. IEEE Transactions on Robotics. 26. 758 - 765. 10.1109/TRO.2010.2052325.`

計算に興味がある場合は、「C. Asymptotical Trajectory Tracking With Orientation Control」セクションでエラーと速度エラーの計算を参照できます。

エラー加速度の計算は、上記の速度の計算に基づいて行われます。次のエラー加速度の計算を参照できます。

![CodeCogsEqn](https://user-images.githubusercontent.com/45468306/169027099-ef15b306-2868-4084-a350-0e2b652c310f.png)

## 入出力

### 入力トピック

| 名前                                   | タイプ                                   | 説明                                   |
| ------------------------------------ | --------------------------------------- | --------------------------------------- |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs::msg::Trajectory` | Planningモジュールからの出力軌道           |
| `/control/command/control_cmd`         | `autoware_control_msgs::msg::Control`  | Controlモジュールからの出力制御コマンド      |
| `/vehicle/status/steering_status`      | `autoware_vehicle_msgs::msg::SteeringReport` | 車両からのステアリング情報                     |
| `/localization/kinematic_state`        | `nav_msgs::msg::Odometry`               | オドメトリからツイストを使用                  |
| `/tf`                                  | `tf2_msgs::msg::TFMessage`             | TFから自車位置を抽出                       |

## 出力トピック

**AutowareのAPIバージョン：** 1.16.0-rc0

| トピック | データタイプ | 説明 |
|---|---|---|
| `/vehicle/localization/current_pose` | `geometry_msgs/PoseStamped` | 自車位置の現在の推定値。このトピックにパブリッシュされるデータは、誤差を軽減するために、センサーデータから後処理やフィルタリングを経て得られたものです。 |
| `/trajectory/planning/lane_waypoints` | `autoware_planning_msgs/Lane` | 計画されたパス内のウェイポイントを格納します。|
| `/trajectory/planning/lattice_waypoints` | `autoware_planning_msgs/Lattice` | 速度と横方向位置（逸脱量）の組み合わせに対する、計画されたパスのウェイポイントを格納します。|
| `/trajectory/planning/optimized_trajectory` | `autoware_planning_msgs/OptimizedTrajectory` | 経路最適化モジュールによって最適化された経路のウェイポイントを格納します。|
| `/planning/planning_flag` | `autoware_planning_msgs/PlanningFlag` | 計画状態に関するフラグを格納します。0：停止、1：フォロー、2：計画中です。|
| `/planning/velocity_plan` | `autoware_planning_msgs/VelocityPlan` | 速度計画モジュールによって計算された車速と加速度の値を格納します。|
| `/control_command` | `autoware_msgs/ControlCommandStamped` | 制御モジュールへ送信される、モーターとステアリングの制御コマンドを格納します。|
| `/localization/odometry` | `nav_msgs/Odometry` | オドメトリ情報。|
| `/localization/estimation` | `geometry_msgs/PoseWithCovarianceStamped` | オドメトリ推定値。|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Velodyneセンサーからの点群。|
| `/image/front_camera/image_raw` | `sensor_msgs/Image` | フロントカメラからの生の画像データ。|
| `/image/rear_camera/image_raw` | `sensor_msgs/Image` | リアカメラからの生の画像データ。|

| 名称                                    | タイプ                                                     | 説明                                             |
| --------------------------------------- | -------------------------------------------------------- | ---------------------------------------------------- |
| `/control_performance/performance_vars` | control_performance_analysis::msg::ErrorStamped          | パフォーマンス解析の結果                          |
| `/control_performance/driving_status`   | control_performance_analysis::msg::DrivingMonitorStamped | 走行ステータス（加速度、ジャークなど）のモニタリング |

### 出力

#### control_performance_analysis::msg::DrivingMonitorStamped

| 名称                         | タイプ  | 説明                                                        |
| ---------------------------- | ----- | ---------------------------------------------------------- |
| `longitudinal_acceleration`  | float | [m/s^2]                                                     |
| `longitudinal_jerk`          | float | [m/s^3]                                                     |
| `lateral_acceleration`       | float | [m/s^2]                                                     |
| `lateral_jerk`               | float | [m/s^3]                                                     |
| `desired_steering_angle`     | float | [rad]                                                        |
| `controller_processing_time` | float | 2つの制御コマンドメッセージ間のタイムスタンプ [ms] |

#### control_performance_analysis::msg::ErrorStamped


| 名称                                       | タイプ  | 説明                                                                                                       |
| ------------------------------------------ | ----- | ----------------------------------------------------------------------------------------------------------------- |
| `lateral_error`                            | float | [m]                                                                                                               |
| `lateral_error_velocity`                   | float | [m/s]                                                                                                           |
| `lateral_error_acceleration`               | float | [m/s^2]                                                                                                         |
| `longitudinal_error`                       | float | [m]                                                                                                               |
| `longitudinal_error_velocity`              | float | [m/s]                                                                                                           |
| `longitudinal_error_acceleration`          | float | [m/s^2]                                                                                                         |
| `heading_error`                            | float | [rad]                                                                                                             |
| `heading_error_velocity`                   | float | [rad/s]                                                                                                         |
| `control_effort_energy`                    | float | [u * R * u^T]                                                                                                     |
| `error_energy`                             | float | lateral_error^2 + heading_error^2                                                                                 |
| `value_approximation`                      | float | V = xPx' ; 自車位置に基づく状態空間のパフォーマンス指数を近似した値                                                        |
| `curvature_estimate`                       | float | [1/m]                                                                                                           |
| `curvature_estimate_pp`                    | float | [1/m]                                                                                                           |
| `vehicle_velocity_error`                   | float | [m/s]                                                                                                           |
| `tracking_curvature_discontinuity_ability` | float | 曲率変化を追従する能力を測定 [`abs(delta(curvature)) / (1 + abs(delta(lateral_error))`] |

## パラメータ

| 名称                                    | 型             | 説明                                                           |
| --------------------------------------- | ---------------- | --------------------------------------------------------------------- |
| `curvature_interval_length`              | double           | current curvature 推定に使用                                            |
| `prevent_zero_division_value`            | double           | ゼロ除算を回避するための値（デフォルトは `0.001`）                |
| `odom_interval`                          | unsigned integer | odom メッセージ間のインターバル（より滑らかな曲線には大きくする） |
| `acceptable_max_distance_to_waypoint`    | double           | 軌道ポイントと車両の最大距離 [m]                                 |
| `acceptable_max_yaw_difference_rad`      | double           | 軌道ポイントと車両の最大ヨー角差 [rad]                              |
| `low_pass_filter_gain`                    | double           | ローパスフィルタのゲイン                                              |

## 操作方法

- シミュレーションと制御モジュールを起動した後、`control_performance_analysis.launch.xml` を起動します。
- 走行モニタとエラー変数がトピックに表示されます。
- 結果を視覚化するには、`Plotjuggler` を使用し、レイアウトとして `config/controller_monitor.xml` を使用します。
- レイアウトをインポートした後、以下のトピックを指定します。

> - /localization/kinematic_state
> - /vehicle/status/steering_status
> - /control_performance/driving_status
> - /control_performance/performance_vars

- `Plotjuggler` では、統計情報（最大、最小、平均）値を csv ファイルとしてエクスポートできます。この統計情報を使用して、制御モジュールを比較します。

## 今後の改善

- カットオフ周波数、微分方程式、離散状態空間の更新を使用して LPF を実装します。


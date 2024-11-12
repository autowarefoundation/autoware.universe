# Predicted Path Checker

## 目的

Predicted Path Checkerパッケージは、自律車両が制御モジュールによって生成された予測パスをチェックするために設計されています。このパッケージは、プランニングモジュールが処理できない可能性があり、ブレーキ距離内にある潜在的な衝突を処理します。ブレーキ距離内に衝突が発生した場合、パッケージはシステムに「ERROR」というラベルの診断メッセージを送信して緊急事態を知らせるようアラートし、リファレンストラジェクトリの外部で衝突が発生した場合は、インターフェースを一時停止して車両を停止するように一時停止要求を送信します。

![general-structure.png](images%2Fgeneral-structure.png)

## アルゴリズム

パッケージアルゴリズムは、予測軌跡をリファレンストラジェクトリと環境内の予測オブジェクトに対して評価します。潜在的な衝突をチェックし、必要に応じてそれらを回避するために適切な応答を生成します（緊急停止または一時停止要求）。

### 内部アルゴリズム

![FlowChart.png](images%2FFlowChart.png)

**cutTrajectory() ->** 予測軌跡を入力された長さでカットします。長さは、エゴカーの速度に「trajectory_check_time」パラメータと「min_trajectory_length」を乗じた値で計算されます。

**filterObstacles() ->** 環境内の予測オブジェクトをフィルタリングします。車両の前方になく、予測軌跡から遠く離れたオブジェクトをフィルタリングします。

**checkTrajectoryForCollision() ->** 予測軌跡が予測オブジェクトと衝突するかどうかをチェックします。軌跡点と予測オブジェクトの両方の多角形を計算し、両方の多角形の交差をチェックします。交差がある場合、最も近い衝突点を計算します。多角形とその予測オブジェクトの最も近い衝突点を返します。また、予期しない動作を避けるために、前にフットプリントと交差した予測オブジェクト履歴もチェックします。予測オブジェクト履歴は、それらが「chattering_threshold」秒前に検出された場合にオブジェクトを格納します。

「enable_z_axis_obstacle_filtering」パラメータが true に設定されている場合、予測オブジェクトは「z_axis_filtering_buffer」を使用して Z 軸でフィルタリングされます。オブジェクトが Z 軸と交差しない場合、オブジェクトはフィルタリングされます。

![Z_axis_filtering.png](images%2FZ_axis_filtering.png)

**calculateProjectedVelAndAcc() ->** 予測軌跡の衝突点の軸上の予測オブジェクトの投影速度と加速度を計算します。

**isInBrakeDistance() ->** 停止点がブレーキ距離内にあるかどうかをチェックします。予測オブジェクトに対するエゴカーの相対速度と加速度を取得します。ブレーキ距離を計算し、点がブレーキ距離内にある場合、true を返します。

**isItDiscretePoint() ->** 予測軌跡上の停止点が離散ポイントであるかどうかをチェックします。離散ポイントでない場合、停止はプランニングによって処理される必要があります。

**isThereStopPointOnRefTrajectory() ->** リファレンストラジェクトリ上に停止点があるかどうかをチェックします。停止インデックスの前に停止点がある場合、true を返します。それ以外の場合は false を返し、ノードは一時停止インターフェースを呼び出して車両を停止します。

## 入力

| 名                                  | 型                                             | 説明                                         |
| ------------------------------------- | ------------------------------------------------ | --------------------------------------------------- |
| `~/input/reference_trajectory`        | `autoware_planning_msgs::msg::Trajectory`        | リファレンストライジェクチャリ                                |
| `~/input/predicted_trajectory`        | `autoware_planning_msgs::msg::Trajectory`        | 予測されたトランスジェクチャリ                                |
| `~/input/objects`                     | `autoware_perception_msgs::msg::PredictedObject` | 環境における動的オブジェクト                  |
| `~/input/odometry`                    | `nav_msgs::msg::Odometry`                        | 自車の速度を取得するためのオドメトリメッセージ |
| `~/input/current_accel`               | `geometry_msgs::msg::AccelWithCovarianceStamped` | 現在の加速度                                |
| `/control/vehicle_cmd_gate/is_paused` | `tier4_control_msgs::msg::IsPaused`              | 車両の現在の停止状態                  |

## 出力

- **軌跡プランニング**
  - 車両軌跡生成における不連続性削減
  - 軌跡プランニングモジュール内での経路最適化
- **制御**
  - 加速度と速度の違反における車両挙動の安定性向上
  - ビークルダイナミクスの向上
- **知覚**
  - オブジェクトの検出精度向上
  - 車両周辺環境のより完全な把握
- **マップ**
  - マップ精度の向上
  - より信頼性の高い経路計画
- **ロギング**
  - システムのパフォーマンスの監視を向上させるロギング機能の追加
- **ビジュアライゼーション**
  - `post resampling`における経路可視化の改善
  - 自車位置のより正確な表現

| 名称                                 | 型                                       | 説明                                     |
| ------------------------------------ | ---------------------------------------- | ---------------------------------------- |
| `~/debug/marker`                      | `visualization_msgs::msg::MarkerArray`   | 可視化用マーカー                          |
| `~/debug/virtual_wall`                | `visualization_msgs::msg::MarkerArray`   | 可視化用の仮想の壁のマーカー              |
| `/control/vehicle_cmd_gate/set_pause` | `tier4_control_msgs::srv::SetPause`      | 車両を停止させるための停止サービス       |
| `/diagnostics`                        | `diagnostic_msgs::msg::DiagnosticStatus` | 車両の診断状況                           |

## パラメータ

### ノードパラメータ

| 名称                                | 型     | 説明                                                               | デフォルト値 |
| :---------------------------------- | :------- | :-------------------------------------------------------------------- | :------------ |
| `update_rate`                       | `倍精度浮動小数点` | 更新レート [Hz]                                                  | 10.0          |
| `delay_time`                        | `倍精度浮動小数点` | 緊急対応で考慮される時間遅延 [s]                               | 0.17          |
| `max_deceleration`                  | `倍精度浮動小数点` | 自動運転車両が停止するための最大減速度 [m/s^2]                    | 1.5           |
| `resample_interval`                 | `倍精度浮動小数点` | 軌道の再サンプリング間隔 [m]                                      | 0.5           |
| `stop_margin`                       | `倍精度浮動小数点` | 停止マージン [m]                                                 | 0.5           |
| `ego_nearest_dist_threshold`        | `倍精度浮動小数点` | 自動運転車両の最近接距離閾値 [m]                                | 3.0           |
| `ego_nearest_yaw_threshold`         | `倍精度浮動小数点` | 自動運転車両の最近接ヨー角閾値 [rad]                            | 1.046         |
| `min_trajectory_check_length`       | `倍精度浮動小数点` | メートル単位での最短軌道チェックの長さ [m]                       | 1.5           |
| `trajectory_check_time`             | `倍精度浮動小数点` | 軌道チェック時間 [s]                                              | 3.0           |
| `distinct_point_distance_threshold` | `倍精度浮動小数点` | 異なる点の距離閾値 [m]                                           | 0.3           |
| `distinct_point_yaw_threshold`      | `倍精度浮動小数点` | 異なる点のヨー角閾値 [deg]                                       | 5.0           |
| `filtering_distance_threshold`      | `倍精度浮動小数点` | 距離がこの値より大きい場合、オブジェクトを無視します [m]         | 1.5           |
| `use_object_prediction`             | `ブーリアン` | Trueの場合、ノードはデルタ時間に関連してオブジェクトの自車位置を予測します [-] | true          |

### 衝突チェッカーパラメータ

| 名称                                | 型     | 説明                                                               | デフォルト値 |
| :--------------------------------- | :------- | :---------------------------------------------------------------- | :------------ |
| `width_margin`                     | `double` | 衝突検査における幅マージン [Hz]                                  | 0.2           |
| `chattering_threshold`             | `double` | 衝突検出におけるチャタリングしきい値 [s]                           | 0.2           |
| `z_axis_filtering_buffer`          | `double` | Z軸フィルタリングバッファ [m]                                     | 0.3           |
| `enable_z_axis_obstacle_filtering` | `bool`   | Z軸障害物フィルタリングが有効かどうかを示すフラグ                    | false         |


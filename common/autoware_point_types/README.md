## Autoware Point Type

### 'Sensor Measurements'ポイントタイプ

#### 'Radar Point'

- センサーからのレーダー測定
- 座標系: センサー座標系
- フィールド:
  - `id`: 整数
  - `meas_time`: タイムスタンプ (sec, nanosec)
  - `range`: 測定対象物までの距離 (m)
  - `range_stddev`: 距離の標準偏差 (m)
  - `azimuth`: 測定対象物までの方位角 (rad)
  - `azimuth_stddev`: 方位角の標準偏差 (rad)
  - `elevation`: 測定対象物までの仰角 (rad)
  - `elevation_stddev`: 仰角の標準偏差 (rad)
  - `vx`: 測定対象物の速度 (m/s)
  - `vx_stddev`: 速度の標準偏差 (m/s)

#### 'Camera Point'

- センサーからのカメラ測定
- 座標系: カメラ座標系
- フィールド:
  - `id`: 整数
  - `meas_time`: タイムスタンプ (sec, nanosec)
  - `x`: 測定対象物のx座標 (pixel)
  - `y`: 測定対象物のy座標 (pixel)
  - `width`: 測定対象物の幅 (pixel)
  - `height`: 測定対象物の高さ (pixel)
  - `orientation`: 測定対象物の向き (rad)
  - `object_type`: 測定対象物のタイプ (車, 歩行者, 自転車など)

#### 'Lidar Point'

- センサーからのLiDAR測定
- 座標系: LiDAR座標系
- フィールド:
  - `id`: 整数
  - `meas_time`: タイムスタンプ (sec, nanosec)
  - `x`: 測定対象物のx座標 (m)
  - `y`: 測定対象物のy座標 (m)
  - `z`: 測定対象物のz座標 (m)
  - `intensity`: 測定対象物の反射強度
  - `ring`: 測定対象物のLiDARスキャンラインの番号

### 'Prediction'ポイントタイプ

#### 'Predicted Object'

- Planningコンポーネントによる予測された測定対象物
- 座標系: 世界座標系
- フィールド:
  - `id`: 整数
  - `pred_time`: 予測されたタイムスタンプ (sec, nanosec)
  - `x`: 予測された測定対象物のx座標 (m)
  - `y`: 予測された測定対象物のy座標 (m)
  - `z`: 予測された測定対象物のz座標 (m)
  - `x_stddev`: 予測された測定対象物のx座標の標準偏差 (m)
  - `y_stddev`: 予測された測定対象物のy座標の標準偏差 (m)
  - `z_stddev`: 予測された測定対象物のz座標の標準偏差 (m)
  - `vx`: 予測された測定対象物の速度 (m/s)
  - `vy`: 予測された測定対象物の速度 (m/s)
  - `vz`: 予測された測定対象物の速度 (m/s)
  - `yaw`: 予測された測定対象物の向き (rad)
  - `yaw_stddev`: 予測された測定対象物の向きの標準偏差 (rad)
  - `length`: 予測された測定対象物の長さ (m)
  - `width`: 予測された測定対象物の幅 (m)
  - `height`: 予測された測定対象物の高さ (m)
  - `object_type`: 予測された測定対象物のタイプ (車, 歩行者, 自転車など)
  - `existence_probability`: 予測された測定対象物の存在確率

### 'Trajectory'ポイントタイプ

#### 'Predicted Trajectory'

- Planningコンポーネントによる予測パス
- 座標系: 世界座標系
- フィールド:
  - `id`: 整数
  - `start_time`: パスの開始時刻 (sec, nanosec)
  - `points`: `'Predicted Object'`ポイントの配列

#### 'Ego Vehicle Motion'

- Planningコンポーネントによる自車位置の予測
- 座標系: 世界座標系
- フィールド:
  - `id`: 整数
  - `meas_time`: タイムスタンプ (sec, nanosec)
  - `x`: 予測された自車位置x座標 (m)
  - `y`: 予測された自車位置y座標 (m)
  - `z`: 予測された自車位置z座標 (m)
  - `yaw`: 予測された自車位置の向き (rad)
  - `vx`: 予測された自車速度 (m/s)
  - `vy`: 予測された自車速度 (m/s)
  - `vz`: 予測された自車速度 (m/s)

### 'Control'ポイントタイプ

#### 'Control Command'

- PlanningコンポーネントからControlコンポーネントへの制御コマンド
- 座標系: 世界座標系
- フィールド:
  - `id`: 整数
  - `meas_time`: タイムスタンプ (sec, nanosec)
  - `acceleration`: 要求される加速度 (m/s²)
  - `steering_angle`: 要求される操舵角 (rad)
  - `velocity`: 要求される速度 (m/s)
  - `acceleration_limit`: 加速度制限 (m/s²)
  - `steering_angle_limit`: 操舵角制限 (rad)
  - `velocity_limit`: 速度制限 (m/s)
  - `v_longitudinal_ jerk_limit`: 縦加速度逸脱量制限 (m/s³)
  - `v_lateral_ jerk_limit`: 横加速度逸脱量制限 (m/s³)
  - `steer_jerk_limit`: 操舵角逸脱量制限 (rad/s³)
  - `max_deceleration`: 最大減速度 (m/s²)
  - `min_deceleration`: 最小減速度 (m/s²)
  - `max_lateral_acceleration`: 最大横加速度 (m/s²)
  - `max_steer_acceleration`: 最大操舵角加速度 (m/s²)
  - `max_steer_angle_rate`: 最大操舵角速度 (rad/s)


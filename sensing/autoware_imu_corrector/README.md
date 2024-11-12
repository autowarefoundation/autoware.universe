# autoware_imu_corrector

## imu_corrector

`imu_corrector_node`はimuデータを補正するノードです。

1. パラメータを読み込んでヨーレートオフセット$b$を補正します。
2. パラメータを読み込んでヨーレートの標準偏差$\sigma$を補正します。

数学的には、以下の式を想定しています。

$$
\tilde{\omega}(t) = \omega(t) + b(t) + n(t)
$$

ここで、$\tilde{\omega}$は観測された角速度、$\omega$は真の角速度、$b$はオフセット、$n$はガウスノイズを表します。
また、$n\sim\mathcal{N}(0, \sigma^2)$と仮定しています。

<!-- TODO(TIER IV): Make this repository public or change the link. -->
<!-- このノードのパラメータとして[deviation_estimator](https://github.com/tier4/calibration_tools/tree/main/localization/deviation_estimation_tools)によって推定された値を使用してください。 -->

### 入力

| 名称     | 型                    | 説明  |
| -------- | ----------------------- | ------------ |
| `~input` | `sensor_msgs::msg::Imu` | 未処理のIMUデータ |

### 自動運転ソフトウェアのドキュメント

#### Planning コンポーネント

Planning コンポーネントは、Autoware の自動運転システムの重要な部分です。周囲環境を認識し、自車位置を推定し、走行経路を生成する役割があります。これらのタスクのために、Planning コンポーネントはさまざまなセンサーからのデータを統合して使用します。

#### 認識コンポーネント

認識コンポーネントは、周囲環境の認識を担当する Planning コンポーネントのサブコンポーネントです。カメラ、レーダー、LiDAR などのセンサーからデータを受け取り、オブジェクトを検出、分類、追跡します。検出されたオブジェクトには、車両、歩行者、自転車、道路標識、建物などが含まれます。

#### 自車位置推定コンポーネント

自車位置推定コンポーネントは、Planning コンポーネントのもう 1 つのサブコンポーネントで、自車位置を推定する役割があります。IMU、GNSS、オドメトリなどのセンサーからデータを受け取り、現在位置と姿勢を推定します。

#### 走行経路生成コンポーネント

走行経路生成コンポーネントは、周囲環境認識と自車位置推定の結果に基づいて走行経路を生成する Planning コンポーネントのサブコンポーネントです。速度、加速度、逸脱量などの制約を考慮して、安全で効率的な走行経路を計算します。

#### 制約検証コンポーネント

制約検証コンポーネントは、走行経路が速度、加速度、逸脱量の制約を満たしているかどうかを検証する Planning コンポーネントのサブコンポーネントです。制約逸脱量を検出して、制約違反がある場合に警報を発信します。

#### `post resampling` コンポーネント

`post resampling` コンポーネントは、Planning コンポーネントのサブコンポーネントで、走行経路を高周波で再サンプリングする役割があります。これにより、車両の動的性能や周囲環境の変化に対応できます。

#### 制御コンポーネント

制御コンポーネントは、Autoware の自動運転システムのもう 1 つの重要な部分で、Planning コンポーネントが生成した走行経路に基づいて車両を制御する役割があります。ステアリング、アクセル、ブレーキなどの作動器を使用します。

#### コントローラーコンポーネント

コントローラーコンポーネントは、制御コンポーネントのサブコンポーネントで、車両の速度、加速度、逸脱量を制御する役割があります。PID 制御や状態空間制御などの制御手法を使用します。

#### アクターコンポーネント

アクターコンポーネントは、制御コンポーネントが生成した制御信号に基づいて車両の作動器を制御する制御コンポーネントのサブコンポーネントです。ステアリングコントローラー、アクセルコントローラー、ブレーキコントローラーなどがあります。

| 名称 | タイプ | 説明 |
|---|---|---|
| `~output` | `sensor_msgs::msg::Imu` | 補正済みIMUデータ |

### パラメータ

| 名称                   | タイプ   | 説明                                         |
| --------------------- | ------ | -------------------------------------------- |
| `angular_velocity_offset_x` | double | imu_linkにおけるロール角速度オフセット [rad/s] |
| `angular_velocity_offset_y` | double | imu_linkにおけるピッチ角速度オフセット [rad/s] |
| `angular_velocity_offset_z` | double | imu_linkにおけるヨー角速度オフセット [rad/s] |
| `angular_velocity_stddev_xx` | double | imu_linkにおけるロール角速度の標準偏差 [rad/s] |
| `angular_velocity_stddev_yy` | double | imu_linkにおけるピッチ角速度の標準偏差 [rad/s] |
| `angular_velocity_stddev_zz` | double | imu_linkにおけるヨー角速度の標準偏差 [rad/s] |
| `acceleration_stddev`    | double | imu_linkにおける加速度の標準偏差 [m/s^2]    |

## gyro_bias_estimator

`gyro_bias_validator`はジャイロのバイアスを検証するノードです。`sensor_msgs::msg::Imu`トピックをサブスクライブし、ジャイロのバイアスが指定範囲内にあるかどうかを確認します。

このノードは、車両が停止している場合に限り、ジャイロデータからバイアスを計算して平均化することに注意してください。

### 入力

| 名称              | 種類                                            | 説明      |
| ----------------- | ----------------------------------------------- | --------- |
| `~/input/imu_raw` | `sensor_msgs::msg::Imu`                         | **未加工** IMU データ |
| `~/input/pose`    | `geometry_msgs::msg::PoseWithCovarianceStamped` | NDT の自車位置         |

入力ポーズは十分に正確であると仮定されています。例えば、NDTを使用する場合、NDTは適切に収束していると仮定します。

現在、Autowareの`pose_source`としてNDT以外のメソッドを使用することは可能ですが、精度の低いメソッドはIMUバイアスの推定には適していません。

将来、ポーズエラーの慎重な実装により、NDTによって推定されたIMUバイアスは検証だけでなくオンラインキャリブレーションにも使用できる可能性があります。

### 出力

| 名称                 | タイプ                                 | 説明                   |
| -------------------- | ------------------------------------ | ----------------------------- |
| `~/output/gyro_bias` | `geometry_msgs::msg::Vector3Stamped` | ジャイロスコープのバイアス [rad/s] |

### パラメータ

このノードは、`imu_corrector.param.yaml`の`angular_velocity_offset_x`、`angular_velocity_offset_y`、`angular_velocity_offset_z`パラメータも使用します。

| 名前                                 | 型   | 説明                                                                                                    |
| -------------------------------------| ------ | -------------------------------------------------------------------------------------------------------- |
| `gyro_bias_threshold`                 | double | ジャイロスコープのバイアスの閾値 [rad/s]                                                                  |
| `timer_callback_interval_sec`         | double | タイマコールバック関数の秒数 [秒]                                                                         |
| `diagnostics_updater_interval_sec`    | double | Diagnostics updater の周期 [秒]                                                                           |
| `straight_motion_ang_vel_upper_limit` | double | 直線運動と見なさない、ヨー角速度の上限 [rad/s]                                                                |


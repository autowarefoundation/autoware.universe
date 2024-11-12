# autoware_ndt_scan_matcher

## 目的

autoware_ndt_scan_matcherはNDTスキャンマッチング手法を使用した位置推定パッケージです。

このパッケージには、次の2つの主な機能があります。

- スキャンマッチングによる位置推定
- モンテカルロ法を使用したROSサービスによる初期位置推定

オプション機能として正則化があります。詳細は後述の正則化の章を参照してください。デフォルトでは無効になっています。

## 入出力

### 入力

| 名称                                | タイプ                                          | 説明                                    |
| ----------------------------------- | --------------------------------------------- | --------------------------------------- |
| `ekf_pose_with_covariance`          | `geometry_msgs::msg::PoseWithCovarianceStamped` | 初期位置                                 |
| `points_raw`                        | `sensor_msgs::msg::PointCloud2`                 | センサポイントクラウド                   |
| `sensing/gnss/pose_with_covariance` | `sensor_msgs::msg::PoseWithCovarianceStamped`   | 正規化項の基本位置                       |

`sensing/gnss/pose_with_covariance` は、正則化が有効な場合にのみ必要です。

### 出力

| 名前                               | タイプ                                            | 説明                                                                                                                              |
| ----------------------------------- | ----------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `ndt_pose`                         | `geometry_msgs::msg::PoseStamped`               | 推定位相                                                                                                                           |
| `ndt_pose_with_covariance`         | `geometry_msgs::msg::PoseWithCovarianceStamped` | 共分散による推定位相                                                                                                           |
| `/diagnostics`                     | `diagnostic_msgs::msg::DiagnosticArray`         | 診断                                                                                                                              |
| `points_aligned`                   | `sensor_msgs::msg::PointCloud2`                 | [デバッグトピック] スキャンマッチングでアラインされた点群                                                                                        |
| `points_aligned_no_ground`         | `sensor_msgs::msg::PointCloud2`                 | [デバッグトピック] スキャンマッチングでアラインされた、地面を除いた点群                                                                              |
| `initial_pose_with_covariance`     | `geometry_msgs::msg::PoseWithCovarianceStamped` | [デバッグトピック] スキャンマッチングで使用される初期位相                                                                                         |
| `multi_ndt_pose`                   | `geometry_msgs::msg::PoseArray`                 | [デバッグトピック] 実時間共分散推定におけるさまざまな初期位相からの推定位相                                             |
| `multi_initial_pose`               | `geometry_msgs::msg::PoseArray`                 | [デバッグトピック] 実時間共分散推定における初期位相                                                                          |
| `exe_time_ms`                      | `tier4_debug_msgs::msg::Float32Stamped`         | [デバッグトピック] スキャンマッチングの実行時間 [ミリ秒]                                                                                      |
| `transform_probability`            | `tier4_debug_msgs::msg::Float32Stamped`         | [デバッグトピック] スキャンマッチングのスコア                                                                                                     |
| `no_ground_transform_probability` | `tier4_debug_msgs::msg::Float32Stamped`         | [デバッグトピック] 地面のない LiDAR スキャンのスコア                                                                       |
| `iteration_num`                    | `tier4_debug_msgs::msg::Int32Stamped`           | [デバッグトピック] スキャンマッチングのイテレーション数                                                                                         |
| `initial_to_result_relative_pose` | `geometry_msgs::msg::PoseStamped`               | [デバッグトピック] 初期点と収束点間の相対位相                                                                                         |
| `initial_to_result_distance`       | `tier4_debug_msgs::msg::Float32Stamped`         | [デバッグトピック] 初期点と収束点間の距離 [メートル]                                                                               |
| `initial_to_result_distance_old`   | `tier4_debug_msgs::msg::Float32Stamped`         | [デバッグトピック] 線形補間で使用される古い 2 つの初期点のうちの 1 つと収束点の距離差 [メートル]                                                |
| `initial_to_result_distance_new`   | `tier4_debug_msgs::msg::Float32Stamped`         | [デバッグトピック] 線形補間で使用される新しい 2 つの初期点のうちの 1 つと収束点の距離差 [メートル]                                                |
| `ndt_marker`                       | `visualization_msgs::msg::MarkerArray`          | [デバッグトピック] デバッグ用のマーカー                                                                                                      |
| `monte_carlo_initial_pose_marker` | `visualization_msgs::msg::MarkerArray`          | [デバッグトピック] 初期位置推定に使用されるパーティクル                                                                              |

### サービス

| Name            | Type                                                            | Description                       |
| --------------- | --------------------------------------------------------------- | --------------------------------- |
| `ndt_align_srv` | `autoware_localization_srvs::srv::PoseWithCovarianceStamped` | 初期姿勢推定サービス             |

## パラメーター

### コアパラメーター

#### フレーム

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/ja/frame.json") }}

#### Sensor Points

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/ja/sensor_points.json") }}

#### Ndt

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/ja/ndt.json") }}

#### 初期姿勢推定

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/ja/initial_pose_estimation.json") }}

#### 検証

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/ja/validation.json") }}

#### スコア推定

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/ja/score_estimation.json") }}

#### 共分散

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/ja/covariance.json") }}

## 正則化

### 概要

この関数は、正規化項をNDT最適化問題に追加し、次のようになります。

$$
\begin{align}
    \min_{\mathbf{R},\mathbf{t}}
    \mathrm{NDT}(\mathbf{R},\mathbf{t})
    +\mathrm{scale\ factor}\cdot \left|
        \mathbf{R}^\top
        (\mathbf{t_{base}-\mathbf{t}})
        \cdot
        \begin{pmatrix}
            1\\
            0\\
            0
        \end{pmatrix}
        \right|^2
\end{align}
$$

ここで、t_baseはGNSSまたは他の手段によって測定されたベース位置です。
NDT(R,t)は純粋なNDTコスト関数を表します。
正則化項は、最適解を車両の**縦方向**のベース位置にシフトさせます。
ベース位置に対する**縦方向**の誤差のみが考慮され、Z軸および横方向の誤差は考慮されません。

**正則化**

正則化項には回転がパラメータとして含まれていますが、最適化を安定化するために、それらに関連付けられた勾配とヘッセ行列を計算しません。
具体的には、勾配は次のように計算されます。

$$
\begin{align}
    &g_x=\nabla_x \mathrm{NDT}(\mathbf{R},\mathbf{t}) + 2 \mathrm{scale\ factor} \cos\theta_z\cdot e_{\mathrm{longitudinal}}
    \\
    &g_y=\nabla_y \mathrm{NDT}(\mathbf{R},\mathbf{t}) + 2 \mathrm{scale\ factor} \sin\theta_z\cdot e_{\mathrm{longitudinal}}
    \\
    &g_z=\nabla_z \mathrm{NDT}(\mathbf{R},\mathbf{t})
    \\
    &g_\mathbf{R}=\nabla_\mathbf{R} \mathrm{NDT}(\mathbf{R},\mathbf{t})
\end{align}
$$

正則化はデフォルトで無効になっています。
使用する場合は、以下のパラメータを編集して有効にしてください。

#### 正則化が利用できる場所

この機能は、GNSS が利用可能な特徴のない道路で有効です。たとえば、

- 橋
- 高速道路
- 農道

以下のように、ベース位置トピックを GNSS 以外のものにリマッピングすることで、これらの外部でも有効にすることができます。

#### 他のベース位置の使用

GNSS 以外の場合は、磁気マーカーやビジュアルマーカーなどから取得した他のグローバル位置トピックを提供できます（利用可能であれば）。
（現在、Autoware はそのような姿勢を与えるノードを提供していません。）正則化にトピックを使用するには、`ndt_scan_matcher.launch.xml` 内で`input_regularization_pose_topic` をトピックにリマッピングする必要があります。
デフォルトでは、`/sensing/gnss/pose_with_covariance` にリマッピングされます。

#### 制限

この機能は、最近サブスクライブした姿勢から線形補間によってベース位置を決定するため、走行速度に対して低い周波数で公開されるトピックは使用できません。
不適切な線形補間は、最適化結果が悪くなる可能性があります。

ベース位置に GNSS を使用する場合、トンネル、屋内、高層ビルの近くで正則化がマイナスの影響を与える可能性があります。
これは、ベース位置が実際の値から大きく離れている場合、NDT スキャンマッチングが不適切な最適位置に収束する可能性があるためです。

### パラメータ

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/sub/ndt_regularization.json") }}

正則化は、GNSS がどのシーンでも適切なベース位置を提供するのに十分な精度とは限らないため、デフォルトで無効になっています。

`scale_factor` が大きすぎると、NDT がベース位置に引き寄せられ、スキャンマッチングが失敗する可能性があります。
逆に、小さすぎると、正則化の利点が失われます。

`scale_factor` を 0 に設定することは、正則化を無効にすることと同じであることに注意してください。

### 例

以下の図は、テストされたマップを示しています。

- 右側はフィーチャがほとんどないマップです。このマップでは、NDTは適切にローカライズできません。

<img src="./media/bridge_map.jpg" alt="drawing" width="300"/> <img src="./media/bridge_map_less_feature.jpg" alt="drawing" width="300"/>

次の図は、それぞれ、フィーチャが少ないマップ上で見積もった軌跡と、正規化が有効なNDTで見積もった軌跡を示しています。
軌跡の色は、フィーチャが豊富なマップで計算された基準軌跡からの誤差（メートル）を示します。

- 左側の図は、純粋なNDTではブリッジに longitudinal エラーが生じ、回復できないことを示しています。
- 右側の図は、正規化により longitudinal エラーが抑制されることを示しています。

<img src="./media/trajectory_without_regularization.png" alt="drawing" width="300"/> <img src="./media/trajectory_with_regularization.png" alt="drawing" width="300"/>

## 動的マップロード

Autowareは `ndt_scan_matcher` 用の動的マップロード機能をサポートしています。この機能を使用することで、NDTは周囲のポイントクラウドマップを `pointcloud_map_loader` に動的に要求し、オンラインでマップを受信して処理します。

この機能を使用すると、`ndt_scan_matcher` は理論的にはメモリ使用量に関して任意の大規模マップを処理できます。（浮動小数点誤差などの他の要因によって制限が存在する可能性があることに注意してください）

<img src="./media/differential_area_loading.gif" alt="drawing" width="400"/>

### 追加インターフェイス

#### 追加出力

| 名 | タイプ | 説明 |
| ---- | ---- | ---- |
| `debug/loaded_pointcloud_map` | `sensor_msgs::msg::PointCloud2` | ローカライゼーション用に使用される点群マップ（デバッグ用） |

#### 追加クライアント

| 名前                | 型                                                   | 説明        |
| ------------------- | ------------------------------------------------------ | ------------------ |
| `client_map_loader` | `autoware_map_msgs::srv::GetDifferentialPointCloudMap` | マップ読み込みクライアント |

### パラメータ

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/sub/dynamic_map_loading.json") }}

### 動的マップロードの注意事項

`ndt_scan_matcher` で動的マップロード機能を使用するには、PCD ファイルをグリッドに分割する必要があります（推奨サイズ：20 [m] x 20 [m]）。

マップが2つ以上の大きなマップ（例：1000 [m] x 1000 [m]）に分割されている場合、動的マップロードは失敗する可能性があります。次のいずれかを提供してください。

- PCD マップファイル 1 つ
- 小さいサイズ（~20 [m]）に分割された複数の PCD マップファイル

Autoware チュートリアルの `sample-map-rosbag` の分割された PCD マップを次に示します。[`sample-map-rosbag_split.zip`](https://github.com/autowarefoundation/autoware.universe/files/10349104/sample-map-rosbag_split.zip)

| PCDファイル | NDTがマップを読み込む方法 |
| :------------: | :------------------: |
| 単一ファイル | 一括（標準） |
| 複数ファイル | 動的 |

## グランドLiDARスキャンなしでのスキャンマッチングスコア

### 概要

これは、グランドLiDARスキャンを使用せずにスキャンマッチングスコアを推定する関数です。このスコアは、現在の局所化パフォーマンスをより正確に反映できます。
[関連するissue](https://github.com/autowarefoundation/autoware.universe/issues/2044)。

### パラメータ

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/sub/score_estimation_no_ground_points.json") }}

## 2Dリアルタイム共分散推定

### 概要

当初、NDTスキャンマッチングの共分散は固定値（FIXED_VALUEモード）です。
そのため、2D共分散（xx、xy、yx、yy）のリアルタイム推定には、LAPLACE_APPROXIMATION、MULTI_NDT、MULTI_NDT_SCOREの3つのモードが用意されています。
LAPLACE_APPROXIMATIONは、NDTスキャンマッチングによって得られたヘシアンのXY（2x2）部分の逆行列を計算し、それを共分散行列として使用します。
一方、MULTI_NDTとMULTI_NDT_SCOREは、複数の初期姿勢からのNDT収束を利用して2D共分散を取得します。
理想的には、複数の初期姿勢の配置は、NDTスコア関数のヘシアン行列によって効率的に制限されます。
この実装では、コードを簡素化するために初期位置の数が固定されています。
MULTI_NDTは、共分散を得るために各初期位置で収束するまで計算を実行する一方、MULTI_NDT_SCOREは最近傍ボクセル変換尤度を使用します。
rviz2でndt_pose_with_covarianceを設定すると、共分散をndtからエラー楕円として見ることができます。
[元論文](https://www.fujipress.jp/jrm/rb/robot003500020435/)。

<img src="./media/calculation_of_ndt_covariance.png" alt="drawing" width="600"/>

この関数は大量の計算リソースを消費すると健全なシステムの挙動を損なう可能性があることに注意してください。

### パラメータ

リアルタイムで2D共分散を計算するための3つの方法があります。covariance_estimation_typeを変更することで方法を選択できます。
initial_pose_offset_modelは、ヘシアン行列の第1主成分の方向に（x、y）=（0、0）を中心に回転します。
initial_pose_offset_model_xとinitial_pose_offset_model_yは同じ数の要素を持つ必要があります。
MULTI_NDT_SCOREモードでは、出力2D共分散のスケールを温度に応じて調整できます。

{{ json_to_markdown("localization/autoware_ndt_scan_matcher/schema/sub/covariance_covariance_estimation.json") }}

## 診断

### scan_matching_status

<img src="./media/diagnostic_scan_matching_status.png" alt="drawing" width="600"/>

| Name                                             | 説明                                                                                        | 警告への遷移条件                                                                                                                                                                                                                                                                                                                               | エラーへの遷移条件 | 推定結果の破棄の有無 ( `skipping_publish_num` に影響) |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------- | ----------------------------------------------------- |
| `topic_time_stamp`                               | 入力トピックのタイムスタンプ                                                              | なし                                                                                                                                                                                                                                                                                                                                                                    | なし                  | いいえ                                                    |
| `sensor_points_size`                             | センサーポイントのサイズ                                                                | サイズが `sensor_points.timeout_sec` より **長い**                                                                                                                                                                                                                                                                                                                    | なし                  | はい                                                     |
| `sensor_points_delay_time_sec`                   | センサーポイントの遅延時間                                                              | 時刻が `sensor_points.timeout_sec` より **長い**                                                                                                                                                                                                                                                                                                                      | なし                  | はい                                                     |
| `is_succeed_transform_sensor_points`             | センサーポイントの変換が成功したかどうか                                                     | なし                                                                                                                                                                                                                                                                                                                                                                    | 失敗                  | はい                                                     |
| `sensor_points_max_distance`                     | センサーポイントの最大距離                                                                | 最大距離が `sensor_points.required_distance` より **短い**                                                                                                                                                                                                                                                                                                                       | なし                  | はい                                                     |
| `is_activated`                                   | ノードが "active" 状態であるかどうか                                                     | "active" 状態ではない                                                                                                                                                                                                                                                                                                                                                  | なし                  | `is_activated` が false の場合、推定は実行されず、 `skipping_publish_num` は 0 に設定されます。 |
| `is_succeed_interpolate_initial_pose`            | 初期姿勢の補間が成功したかどうか                                                       | 失敗。<br>(1) `initial_pose_buffer_` のサイズが **2** より小さい。<br>(2) 初期姿勢とセンサーポイントクラウド間のタイムスタンプの差が `validation.initial_pose_timeout_sec` より **長い**。<br>(3) 線形補間に使用される 2 つ の初期姿勢間の距離の差が `validation.initial_pose_distance_tolerance_m` より **長い** | なし                  | はい                                                     |
| `is_set_map_points`                              | マップポイントが設定されているかどうか                                                   | 設定されていない                                                                                                                                                                                                                                                                                                                                                  | なし                  | はい                                                     |
| `iteration_num`                                  | 整列を計算する回数                                                                      | 回数が `ndt.max_iterations` より **多い**                                                                                                                                                                                                                                                                                                                              | なし                  | はい                                                     |
| `local_optimal_solution_oscillation_num`         | ソリューションが振動していると判断された回数                                              | 回数が **10** より **多い**                                                                                                                                                                                                                                                                                                                                 | なし                  | はい                                                     |
| `transform_probability`                          | マップがセンサーポイントとどれほどよく一致するかを表すスコア                               | スコアが `score_estimation.converged_param_transform_probability` より **小さい** (`score_estimation.converged_param_type` が 0=TRANSFORM_PROBABILITY の場合のみ)                                                                                                                                                                                                    | なし                  | はい                                                     |
| `transform_probability_diff`                     | 現在 `ndt` 最適化のtpスコア差                                                           | なし                                                                                                                                                                                                                                                                                                                                                                    | なし                  | いいえ                                                    |
| `transform_probability_before`                   | 現在 `ndt` 最適化前のtpスコア                                                             | なし                                                                                                                                                                                                                                                                                                                                                                    | なし                  | いいえ                                                    |
| `nearest_voxel_transformation_likelihood`        | マップがセンサーポイントとどれほどよく一致するかを表すスコア                               | スコアが `score_estimation.converged_param_nearest_voxel_transformation_likelihood` より **小さい** (`score_estimation.converged_param_type` が 1=NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD の場合のみ)                                                                                                                                                                         | なし                  | はい                                                     |
| `nearest_voxel_transformation_likelihood_diff`   | 現在 `ndt` 最適化のnvtlスコア差                                                        | なし                                                                                                                                                                                                                                                                                                                                                                    | なし                  | いいえ                                                    |
| `nearest_voxel_transformation_likelihood_before` | 現在 `ndt` 最適化前のnvtlスコア                                                         | なし                                                                                                                                                                                                                                                                                                                                                                    | なし                  | いいえ                                                    |
| `distance_initial_to_result`                     | 収束処理前と後の位置間の距離                                                           | 距離が `validation.initial_to_result_distance_tolerance_m` より **長い**                                                                                                                                                                                                                                                                                                                      | なし                  | いいえ                                                    |
| `execution_time`                                 | 収束処理時間                                                                            | 時刻が `validation.critical_upper_bound_exe_time_ms` より **長い**                                                                                                                                                                                                                                                                                                                       | なし                  | いいえ                                                    |
| `skipping_publish_num`                           | 連続して推定結果を破棄した回数                                                          | 回数が `validation.skipping_publish_num` 以上                                                                                                                                                                                                                                                                                                                         | なし                  | -                                                        |

※`sensor_points_callback`は`trigger_node_service`および`ndt_align_service`と同一コールバックグループを共有しています。結果的に、初期ポーズ推定に時間がかかりすぎると、この診断は古くなる可能性があります。

### initial_pose_subscriber_status

<img src="./media/diagnostic_initial_pose_subscriber_status.png" alt="drawing" width="600"/>

| 名前                           | 説明                                                                                          | 注意の状態への遷移条件                   | エラーの状態への遷移条件                                |
| ------------------------------ | --------------------------------------------------------------------------------------------------- | ----------------------------------------- | -------------------------------------------------------- |
| `topic_time_stamp`             | 入力トピックのタイムスタンプ                                                                   | なし                                         | なし                                                    |
| `is_activated`                 | ノードが「アクティブ」状態であるかどうか                                                              | 「アクティブ」状態以外                         | なし                                                    |
| `is_expected_frame_id`         | 入力フレーム IDが `frame.map_frame`と同一かどうか                                                   | なし                                         | 異なる場合                                             |

### regularization_pose_subscriber_status

<img src="./media/diagnostic_regularization_pose_subscriber_status.png" alt="drawing" width="600"/>

| Name                      | Description                              | Transition condition to Warning | Transition condition to Error |
| ------------------------- | --------------------------------------- | ---------------------------------- | ----------------------------- |
| `topic_time_stamp`         | 入力トピックのタイムスタンプ            | なし                                | なし                          |

### trigger_node_service_status

<img src="./media/diagnostic_trigger_node_service_status.png" alt="図" width="600"/>

| 名称                      | 説明                                              | Warningへの移行条件 | Errorへの移行条件 |
| ------------------------- | --------------------------------------------------- | -------------------------- | --------------------- |
| `service_call_time_stamp` | サービス呼び出しのタイムスタンプ                | なし                            | なし                      |
| `is_activated`            | ノードが「アクティブ」状態か否か                     | なし                            | なし                      |
| `is_succeed_service`      | サービスプロセスの成功または失敗                     | なし                            | なし                      |

※
この診断はサービスが呼び出されたときにのみ発行されるので、初期姿勢推定が完了すると古いものになります。

### ndt_align_service_status

<img src="./media/diagnostic_ndt_align_service_status.png" alt="図" width="600"/>

| 名前                                 | 説明                                                                                                                                                                                                                                    | 警告への遷移条件 | エラーへの遷移条件 |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------- | ------------------- |
| `service_call_time_stamp`            | サービス呼び出しのタイムスタンプ                                                                                                                                                                                                | なし                     | なし                 |
| `is_succeed_transform_initial_pose`  | 初期位置の変換に成功したかどうか                                                                                                                                                                                             | なし                     | エラー               |
| `is_need_rebuild`                   | マップを再構築する必要があるかどうか。まだマップがロードされていない場合、または「`distance_last_update_position_to_current_position encounters`」がエラー状態の場合、マップの再構築が必要と見なされ、`is_need_rebuild` が `True` になります | なし                     | なし                 |
| `maps_size_before`                  | マップの更新前のマップ数                                                                                                                                                                                                           | なし                     | なし                 |
| `is_succeed_call_pcd_loader`        | pcd_loader サービスの呼び出しに成功したかどうか                                                                                                                                                                            | エラー                   | なし                 |
| `maps_to_add_size`                   | 追加されるマップ数                                                                                                                                                                                                                 | なし                     | なし                 |
| `maps_to_remove_size`               | 削除されるマップ数                                                                                                                                                                                                                 | なし                     | なし                 |
| `map_update_execution_time`         | マップの更新にかかる時間                                                                                                                                                                                                            | なし                     | なし                 |
| `maps_size_after`                   | マップの更新後のマップ数                                                                                                                                                                                                           | なし                     | なし                 |
| `is_updated_map`                    | マップが更新されたかどうか。マップの更新を実行できなかった場合、またはマップを更新する必要がなかった場合、`False` になります。                                                                                    | なし                     | `is_updated_map` が `False` で `is_need_rebuild` が `True` |
| `is_set_map_points`                 | マップポイントが設定されているかどうか                                                                                                                                                                                          | 設定されていない       | なし                 |
| `is_set_sensor_points`              | センサーポイントが設定されているかどうか                                                                                                                                                                                        | 設定されていない       | なし                 |
| `best_particle_score`               | 粒子のベストスコア                                                                                                                                                                                                           | なし                     | なし                 |
| `is_succeed_service`                | サービスの処理が成功したかどうか                                                                                                                                                                                             | エラー                   | なし                 |

※
この診断はサービスが呼び出され時にのみ発行されるため、初期ポーズ推定が完了すると古くなります。

### マップ更新ステータス

<img src="./media/diagnostic_map_update_status.png" alt="drawing" width="600"/>

| 項目                                            | 説明                                                                                                                                                                                                                                                | 警告状態への移行条件 | エラー状態への移行条件                                                                                                          |
| -------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `timer_callback_time_stamp`                  | `timer_callback` の呼び出しタイムスタンプ                                                                                                                                                                                                           | なし                     | なし                                                                                                                                     |
| `is_activated`                               | ノードが "activate" 状態であるかどうか                                                                                                                                                                                                                        | "activate" 状態でない  | なし                                                                                                                                     |
| `is_set_last_update_position`              | `last_update_position` が設定されているかどうか                                                                                                                                                                                                                 | 設定されてない          | なし                                                                                                                                     |
| `distance_last_update_position_to_current_position` | `last_update_position` から現在の位置までの距離                                                                                                                                                                                          | なし                     | (距離 + `dynamic_map_loading.lidar_radius`) が `dynamic_map_loading.map_radius` より **大きい**                             |
| `is_need_rebuild`                             | マップを再構築する必要があるかどうか。まだマップが読み込まれていなかったり、`distance_last_update_position_to_current_position` がエラー状態を検出したりすると、マップを再構築する必要があり、`is_need_rebuild` が `True` になる。 | なし                     | なし                                                                                                                                     |
| `maps_size_before`                            | マップ更新前のマップの数                                                                                                                                                                                                                          | なし                     | なし                                                                                                                                     |
| `is_succeed_call_pcd_loader`                 | `pcd_loader` サービスを呼び出すことに成功したかどうか                                                                                                                                                                                            | 失敗                      | なし                                                                                                                                     |
| `maps_to_add_size`                            | 追加するマップの数                                                                                                                                                                                                                             | なし                     | なし                                                                                                                                     |
| `maps_to_remove_size`                          | 削除するマップの数                                                                                                                                                                                                                             | なし                     | なし                                                                                                                                     |
| `map_update_execution_time`                  | マップ更新の時間                                                                                                                                                                                                                          | なし                     | なし                                                                                                                                     |
| `maps_size_after`                             | マップ更新後のマップの数                                                                                                                                                                                                                         | なし                     | なし                                                                                                                                     |
| `is_updated_map`                              | マップが更新されたかどうか。マップの更新が実行できなかったか、マップを更新する必要がなかった場合、`False` になる                                                                                                                                           | なし                     | `is_updated_map` が `False` だが `is_need_rebuild` が `True`                                           |


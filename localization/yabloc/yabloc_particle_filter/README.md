## yabLoc_particle_filter

このパッケージには、パーティクルフィルタに関連する実行可能ノードが含まれています。

- [particle_predictor](#particle_predictor)
- [gnss_particle_corrector](#gnss_particle_corrector)
- [camera_particle_corrector](#camera_particle_corrector)

## particle_predictor

### 目的

- このノードは、パーティクルの予測更新と再サンプリングを実行します。
- それは、Correctorノードによって決定されるパーティクルの重みを遡及的に反映します。

### 入出力

#### 入力
- `/localization/particle_cloud/sample` (ParticleCloud)

#### 出力
- `/localization/particle_cloud/prediction_sample` (ParticleCloud)
- `/localization/particle_cloud/prediction_weight` (ParticleWeightArray)

### パラメータ

| パラメータ | 説明 | 型 | デフォルト |
| --- | --- | --- | --- |
| `resampling` | 仮説の再サンプリングを実行するかどうか | bool | true |
| `resampling_threshold` | 仮説が再サンプリングされる前の重みの最小しきい値 | double | 0.001 |
| `min_num_particles` | 再サンプリング後の最小パーティクル数 | int | 100 |
| `max_num_particles` | 再サンプリング後の最大パーティクル数 | int | 10000 |
| `temporal_variance` | `'post resampling'`仮説の生成に対する時系列的な分散 | double | 0.0 |
| `translational_variance` | `'post resampling'`仮説の生成に対する並進的な分散 | double | 0.0 |
| `rotational_variance` | `'post resampling'`仮説の生成に対する回転的な分散 | double | 0.0 |

## gnss_particle_corrector

### 目的

- このノードは、GNSS測定値に基づいてパーティクルの重みを修正します。

### 入出力

#### 入力
- `/localization/particle_cloud/ground_truth` (ParticleCloud)
- `/localization/particle_cloud/prediction_sample` (ParticleCloud)
- `/localization/gnss/odometry` (Odometry)

#### 出力
- `/localization/particle_cloud/correction_weight` (ParticleWeightArray)

### パラメータ

| パラメータ | 説明 | 型 | デフォルト |
| --- | --- | --- | --- |
| `gnss_model` | GNSSの測定モデル | string | `'gnss_sim'` |
| `gnss_device_model` | 使用するGNSSデバイスモデル | string | `'ublox_f9p'` |
| `gnss_max_age` | GNSS測定値の最大許容時間 | double | 1.0 |

## camera_particle_corrector

### 目的

- このノードは、カメラ測定値に基づいてパーティクルの重みを修正します。

### 入出力

#### 入力
- `/ localization / particle_cloud / ground_truth` (ParticleCloud)
- `/ localization / particle_cloud / prediction_sample` (ParticleCloud)
- `/localization/camera/detection` (DetectionArray)

#### 出力
- `/localization/particle_cloud/correction_weight` (ParticleWeightArray)

### パラメータ

| パラメータ | 説明 | 型 | デフォルト |
| --- | --- | --- | --- |
| `camera_model` | カメラの測定モデル | string | `'camera_sim'` |
| `camera_device_model` | 使用するカメラデバイスモデル | string | `'realsense_d435'` |
| `camera_max_age` | カメラ測定値の最大許容時間 | double | 1.0 |
| `detection_distance_threshold` | 検出とパーティクルの位置の距離の最大しきい値 | double | 1.0 |

| 名前                          | 種類                                              | 説明                                                 |
| ----------------------------- | ------------------------------------------------ | ----------------------------------------------------- |
| `input/initialpose`           | `geometry_msgs::msg::PoseWithCovarianceStamped`  | パーティクルの初期位置を指定                           |
| `input/twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | 予測更新の線速度と角速度                              |
| `input/height`                | `std_msgs::msg::Float32`                         | 地形高                                                 |
| `input/weighted_particles`    | `yabloc_particle_filter::msg::ParticleArray`     | 補正ノードによって重み付けされたパーティクル               |

#### 出力

**自動運転ソフトウェア**

Autowareの自動運転ソフトウェアは、Perception、Planning、Controlの3つの主要コンポーネントで構成されています。

**Perception**

Perceptionコンポーネントは、カメラ、LiDAR、レーダーなどのセンサーからのデータを処理し、「点群」と「カメラ画像」を生成します。これらのデータを使用して、障害物（車両、歩行者、自転車など）、道路標識、車線マーキングなどの周辺環境を認識します。

**Planning**

Planningコンポーネントは、Perceptionから得られたデータを基に、安全で効率的な走行経路を生成します。経路生成では、以下の要因が考慮されます。

* 周囲環境の認識
* 障害物回避
* 車両の運動力学（速度、加速度）
* 交通規則

**Control**

Controlコンポーネントは、Planningから生成された経路に従って、車両を制御します。これには、ステアリング、アクセル、ブレーキの操作が含まれます。

**追加機能**

* **Lane Keeping Assist (LKA):** 車線を維持するアシスタント
* **Adaptive Cruise Control (ACC):** 前方車両との車間距離を維持するクルーズコントロール
* **Emergency Braking:** 衝突の可能性がある場合に自動的にブレーキをかける機能

**評価**

Autowareの性能は、さまざまな指標を使用して評価できます。

* **Planning評価:**
    * 障害物逸脱量（velocity, acceleration）
    * 車線逸脱量
    * 安全距離
* **Control評価:**
    * 追従精度
    * 'post resampling'精度
    * 車両の安定性

**使用例**

Autowareの自動運転ソフトウェアは、自動運転車両、ロボタクシー、ラストワンマイル配送などのさまざまな用途で使用できます。

| 名前 | 内容 | 説明 |
|---|---|---|
| `output/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | 粒子中心と共分散 |
| `output/pose` | `geometry_msgs::msg::PoseStamped` | 粒子中心と共分散 |
| `output/predicted_particles` | `yabloc_particle_filter::msg::ParticleArray` | 予測ノードで重み付けされた粒子 |
| `debug/init_marker` | `visualization_msgs::msg::Marker` | 初期位置のデバッグ用可視化 |
| `debug/particles_marker_array` | `visualization_msgs::msg::MarkerArray` | 粒子の可視化。`visualize` が true の場合に配信 |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/predictor.schema.json") }}

### サービス

| 名称                  | 種類                      | 説明                                                  |
| -------------------- | ------------------------ | ---------------------------------------------------- |
| `yabloc_trigger_srv` | `std_srvs::srv::SetBool` | yabloc推定の有効化および無効化 |

## gnss_particle_corrector

### 目的

- このノードは、GNSSを使用して粒子重みを推定します。
- 2種類の入力に対応しています。`ublox_msgs::msg::NavPVT`と`geometry_msgs::msg::PoseWithCovarianceStamped`です。

### 入出力

#### 入力
- ublox\_msgs::NavPVT：[GNSS情報](https://docs.autoware.io/en/latest/autoware.auto/msgs/ublox_msgs/message/NavPVT.html)
- geometry\_msgs::PoseWithCovarianceStamped：[自車位置](https://docs.autoware.io/en/latest/autoware.auto/msgs/geometry_msgs/message/PoseWithCovarianceStamped.html)

| 名前                           | タイプ                                             | 説明                                                    |
| ------------------------------ | ------------------------------------------------ | ------------------------------------------------------- |
| `input/height`               | `std_msgs::msg::Float32`                          | 地上高度                                              |
| `input/predicted_particles`  | `yabloc_particle_filter::msg::ParticleArray`       | 推定パーティクル                                      |
| `input/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | GNSS測定 (use_ublox_msgがfalseの場合に使用)         |
| `input/navpvt`               | `ublox_msgs::msg::NavPVT`                          | GNSS測定 (use_ublox_msgがtrueの場合に使用)         |

**自動運転ソフトウェアの設計に関するドキュメント**

**Planningコンポーネント**

**状態推定と構想**

* 最新の自車位置、周囲環境のステータス、予定ルートを把握するための状態推定
* 障害物の検知、予測、および分類に基づいたパス計画の構想

**動作計画**

* 経路最適化と障害物回避のためのグローバルプランナー
* 局所プランナーによるリアルタイム動作計画の生成

**制御**

* 車両の動的特性を考慮した制御器の設計
* 経路追従、速度制御、障害物回避のための閉ループ制御

**シミュレーションとテスト**

* 仮想環境を使用した自動運転システムのシミュレーション
* 実際のテスト走路での実車テスト

**システムアーキテクチャ**

* モジュール化されたソフトウェアアーキテクチャ
* さまざまなセンサーとアクチュエーターとのインターフェース

**Autowareソフトウェアスタック**

* オープンソースの自動運転ソフトウェアプラットフォーム
* Planning、制御、センシング、シミュレーションのためのモジュールを提供

**主要な概念**

* **'post resampling'**：状態推定後の予測された経路
* **速度逸脱量**：目標速度からの逸脱
* **加速度逸脱量**：目標加速度からの逸脱
* **制御限界**：車両の物理的な制約による制御器の入出力の限界
* **Lanelet2**：道路環境を表現するためのデータ構造

| 名前                           | タイプ                                         | 説明                                                   |
| ------------------------------ | -------------------------------------------- | -------------------------------------------------------- |
| `output/weighted_particles`    | `yabloc_particle_filter::msg::ParticleArray` | 重み付けられた粒子                                     |
| `debug/gnss_range_marker`      | `visualization_msgs::msg::MarkerArray`       | GNSS の重み分布                                       |
| `debug/particles_marker_array` | `visualization_msgs::msg::MarkerArray`       | 粒子のビジュアライゼーション. `visualize` が真の場合に公開される |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/gnss_particle_corrector.schema.json") }}

## camera_particle_corrector

### 目的

- このノードは GNSS を使用してパーティクルの重みを見積もります。

### 入出力

#### 入力

| 名称                                | タイプ | 説明 |
| --------------------------------- | ----------------------------------------- | --------------------------------------------------------------- |
| `input/predicted_particles` | `yabloc_particle_filter::msg::ParticleArray` | 予測粒子 |
| `input/ll2_bounding_box` | `sensor_msgs::msg::PointCloud2` | 路面標示を線分に変換したもの |
| `input/ll2_road_marking` | `sensor_msgs::msg::PointCloud2` | 路面標示を線分に変換したもの |
| `input/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | 投影線分 |
| `input/pose` | `geometry_msgs::msg::PoseStamped` | 自車位置周辺のエリアマップを取得するための参照 |

#### 出力

Autoware Planning 2.x のモジュール開発チュートリアル

このチュートリアルでは、Planning 2.x のモジュール開発に関するエンドツーエンドのプロセスをご紹介します。必要なスキルと知識、および Planning 2.x でモジュールを開発するための推奨アプローチについて説明します。

### 必要条件

* C++ の中級レベルの知識
* ROS の基本的な知識
* Autoware の基本的な知識

###推奨アプローチ

Planning 2.x でモジュールを開発するには、以下の推奨アプローチに従うことをお勧めします。

1. **要件の定義:** モジュールの目的、入力、および出力について明確に定義します。
2. **インターフェースの設計:** モジュールと外部コンポーネントとのインターフェースを設計します。
3. **モジュールの実装:** モジュールのアルゴリズムとロジックを実装します。
4. **テスト:** 単体テスト、統合テスト、システムテストを通じてモジュールをテストします。
5. **ドキュメント化:** モジュールのインターフェース、実装、テストについて文書化します。

### モジュール構成

Planning 2.x のモジュールは、以下のような構成になっています。

* **リソース:** モジュールによって使用される設定ファイルやパラメータです。
* **インターフェース:** モジュールと外部コンポーネントとのやり取りに使用されるクラスと関数です。
* **アルゴリズム:** モジュールの内部ロジックを実装するコードです。
* **テスト:** モジュールを検証するためのテストケースです。
* **ドキュメント:** モジュールのインターフェース、実装、テストについての説明です。

### モジュールの例

以下に、Planning 2.x で実装されているモジュールの例をいくつか示します。

* **Local Planner:** 自車位置から近接将来の軌道を生成します。
* **Global Planner:** 長期的な将来の軌道を生成します。
* **Behavior Planner:** 自車の動作を決定します。
* **Prediction Module:** 他者の動きを予測します。

### モジュール開発の手順

Planning 2.x でモジュールを開発するには、以下の手順に従います。

1. **Planning モジュールテンプレートを複製する:** Planning リポジトリからモジュールテンプレートを複製します。
2. **モジュールに名前を付ける:** モジュールを適切な名前に変更します。
3. **インターフェースを編集する:** モジュールのインターフェースを編集して、独自の要件に合わせます。
4. **アルゴリズムを実装する:** モジュールのアルゴリズムを実装します。
5. **テストを追加する:** モジュールを検証するためのテストを追加します。
6. **リソースを追加する:** モジュールによって使用されるリソースを追加します。
7. **ドキュメントを追加する:** モジュールのインターフェース、実装、テストについてのドキュメントを追加します。
8. **モジュールをコンパイルしてインストールする:** モジュールをコンパイルして Autoware にインストールします。

### モジュールのメンテナンス

Planning 2.x のモジュールをメンテナンスするには、以下のベストプラクティスに従うことをお勧めします。

* **コードの変更を記録する:** コードの変更をコミットログに記録します。
* **継続的な統合を使用する:** 変更があると自動的にモジュールをテストおよびビルドする継続的な統合システムを使用します。
* **モジュールを更新する:** Planning の新しいバージョンがリリースされたら、モジュールを更新します。

### サポート

Planning 2.x のモジュール開発に関するサポートについては、[Autoware フォーラム](https://forum.autoware.ai/) にアクセスしてください。

| 名前                              | タイプ                                            | 説明                                                      |
| -----------------                | ----------------------------------------------- | ---------------------------------------------------------- |
| `output/weighted_particles`         | `yabloc_particle_filter::msg::ParticleArray` | ウェイト付き粒子                                            |
| `debug/cost_map_image`              | `sensor_msgs::msg::Image`                     | lanelet2 から生成されたコストマップ                        |
| `debug/cost_map_range`              | `visualization_msgs::msg::MarkerArray`        | コストマップ境界                                            |
| `debug/match_image`                 | `sensor_msgs::msg::Image`                     | 投影線分画像                                               |
| `debug/scored_cloud`                | `sensor_msgs::msg::PointCloud2`               | ウェイト付き3D線分                                           |
| `debug/scored_post_cloud`           | `sensor_msgs::msg::PointCloud2`               | ウェイト付き3D線分（不確実なもの）                            |
| `debug/state_string`                | `std_msgs::msg::String`                       | ノード状態を表す文字列                                    |
| `debug/particles_marker_array`      | `visualization_msgs::msg::MarkerArray`        | 粒子ビジュアライゼーション.`visualize` が True の場合に公開 |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/camera_particle_corrector.schema.json") }}

### サービス

| 名         | 種類                     | 説明                               |
| ------------ | ------------------------ | ----------------------------------------- |
| `switch_srv` | `std_srvs::srv::SetBool` | 補正の有効化と無効化                   |


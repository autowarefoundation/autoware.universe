# yabloc_image_processing

このパッケージには画像処理に関連するいくつかの実行可能ノードが含まれています。

- [line_segment_detector](#line_segment_detector)
- [graph_segmentation](#graph_segmentation)
- [segment_filter](#segment_filter)
- [undistort](#undistort)
- [lanelet2_overlay](#lanelet2_overlay)
- [line_segments_overlay](#line_segments_overlay)

## line_segment_detector

### 目的

このノードはグレースケール画像から全ての直線を抽出します。

### 入力/出力

#### 入力

| 名前              | タイプ                    | 説明       |
| ----------------- | ------------------------ | ----------------- |
| `input/image_raw` | `sensor_msgs::msg::Image` | 非歪画像 |

## 自動運転ソフトウェアドキュメント（マークダウン形式）

### 目次

- [Planningモジュール](#planningモジュール)
- [Behavior Planning](#behavior-planning)
- [Trajectory Planning](#trajectory-planning)
- [Control](#control)
- [Localization](#localization)
- [Perception](#perception)
- [Integration](#integration)

### Planningモジュール

Planningモジュールは、自車位置を出発点として、目標の到着点まで安全で効率的な経路を作成する責任を負います。Planningモジュールは、次に示す2つのサブモジュールで構成されています。

#### Behavior Planning

Behavior Planningサブモジュールは、任意の外部入力なしで自車に安全で効率的な目的地への経路を計画します。このサブモジュールは、経路計画アルゴリズム、経路評価機能、および衝突回避モジュールで構成されます。

#### Trajectory Planning

Trajectory Planningサブモジュールは、外部入力（衝突回避モジュールやLocalizationモジュールなど）に基づいて、自車の具体的な経路を計画します。このサブモジュールは、Trajectory Generator、`post resampling`アルゴリズム、および低速走行アルゴリズムで構成されます。

### Behavior Planning

Behavior Planningサブモジュールは、入力として現在のマップと自車位置を受け取ります。サブモジュールは、衝突の可能性がある障害物を回避しながら、経路計画アルゴリズムを使用して目的地までの安全で効率的な経路を計画します。

### Trajectory Planning

Trajectory Planningサブモジュールは、Behavior Planningサブモジュールによって作成された経路を受け取ります。サブモジュールは、`post resampling`アルゴリズムを使用して経路を滑らかにし、低速走行アルゴリズムを使用して交差点などの低速走行シナリオを処理します。

### Control

Controlモジュールは、計画された経路に基づいて、車両の速度とステアリングを制御します。このモジュールは、PIDコントローラー、状態オブザーバー、およびActuationモジュールで構成されます。

### Localization

Localizationモジュールは、自車位置を決定します。このモジュールは、GPS、IMU、カメラなどのセンサーからデータを受け取ります。

### Perception

Perceptionモジュールは、周囲の環境を感知します。このモジュールは、カメラ、レーダー、LiDARなどのセンサーからデータを受け取ります。

### Integration

Integrationモジュールは、Planningモジュール、Controlモジュール、Localizationモジュール、およびPerceptionモジュールを統合します。このモジュールは、センサーデータを統合し、モジュール間で通信を行います。

Autowareのアーキテクチャがご理解いただけたかと思います。質問やご提案がありましたら、お気軽にお問い合わせください。

| 名称                                | タイプ                             | 説明                                                                                                                             |
| ------------------------------------ | --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `output/image_with_line_segments`     | `sensor_msgs::msg::Image`        | 線分を強調表示した画像                                                                                                                  |
| `output/line_segments_cloud`          | `sensor_msgs::msg::PointCloud2`   | 点群としての検出された線分。各点は x、y、z、normal_x、normal_y、normal_z を含み、z と normal_z は常に空です。 |

## graph_segmentation

### 目的

このノードは、[グラフベースセグメンテーション](https://docs.opencv.org/4.5.4/dd/d19/classcv_1_1ximgproc_1_1segmentation_1_1GraphSegmentation.html)によって道路領域を抽出します。

### 入力 / 出力

#### 入力

| Name              | Type                      | Description       |
| ----------------- | ------------------------- | ----------------- |
| `input/image_raw` | `sensor_msgs::msg::Image` | 歪み補正された画像 |

## 自動運転ソフトウェア仕様書

### 計画コンポーネント

#### 動作概要

計画コンポーネントは、車両が目的地に安全かつ効率的に移動するための経路と操作を生成する。

#### 入出力

* **入力:**
    * 自車位置
    * マップ
    * 障害物検出
* **出力:**
    * 経路
    * 加速
    * ハンドリング

#### アルゴリズム

計画コンポーネントは、以下を含むさまざまなアルゴリズムを使用している:

* **経路計画:** A*アルゴリズム、Dijkstraアルゴリズム
* **動作計画:** Model Predictive Control (MPC)、最適制御
* **障害物回避:** Velocity Obstacles法、Dynamic Window Approach (DWA)

#### パフォーマンス指標

* **経路の安全性:** 障害物逸脱量、速度逸脱量、加速度逸脱量の最小化
* **経路の効率性:** 移動時間の最小化、燃料消費の最小化
* **計算時間:** リアルタイム要件の遵守

#### システムアーキテクチャ

計画コンポーネントは、以下を含むモジュール式アーキテクチャで設計されている:

* **経路プランナー:** 経路を生成する
* **動作プランナー:** 車両の操作を生成する
* **障害物検出と回避:** 障害物を検出し、それらを回避する経路と操作を生成する

#### 統合

計画コンポーネントは、Autowareシステムの他のコンポーネントと統合されている。これらには、以下が含まれる:

* センサーインターフェース: 障害物検出データを計画コンポーネントに提供する
* 制御インターフェース: 計画コンポーネントから車両の制御系に動作コマンドを送信する
* HMIインターフェース: 計画経路と車両の操作をユーザーに表示する

#### テストと検証

計画コンポーネントは、シミュレーション環境と実車テストで徹底的にテストされている。このテストには以下が含まれる:

* **シミュレーション:** さまざまなシナリオ下での計画コンポーネントの動作の検証
* **実車テスト:** 実際の世界での計画コンポーネントの性能評価

#### リリースノート

* **バージョ ン1.0:** 初期リリース
* **バージョン1.1:** 障害物検出と回避の改善
* **バージョン1.2:** パフォーマンスの向上とバグ修正

| 名称                     | 型                      | 説明                                                  |
| ------------------------ | ------------------------- | -------------------------------------------------------- |
| `output/mask_image`      | `sensor_msgs::msg::Image` | 路面領域として特定されたマスクされたセグメントを含むイメージ |
| `output/segmented_image` | `sensor_msgs::msg::Image` | 可視化用のセグメントイメージ                          |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_image_processing/schema/graph_segment.schema.json") }}

## segment_filter

### 目的

このノードは、graph_segmentとlsdの結果を統合し、道路表面の標示を抽出します。

### 入出力

#### 入力
- `lane_detected_raw` : レーン検出の生データ
- `line_detected_raw` : 線検出の生データ
- `current_pose` : 自車位置

| 名前 | タイプ | 説明 |
|----|----|----|
| `input/line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | 検出された線分 |
| `input/mask_image` | `sensor_msgs::msg::Image` | 路面領域と判断されたマスクされたセグメントを持つ画像 |
| `input/camera_info` | `sensor_msgs::msg::CameraInfo` | カメラの非歪み情報 |

#### 出力

**自動運転ソフトウェアのアーキテクチャ**

**目的**

このドキュメントでは、Autowareのアーキテクチャとその主なコンポーネントについて説明します。

**アーキテクチャ**

Autowareは、モジュール化されたアーキテクチャに基づいています。このアーキテクチャは、以下の主要コンポーネントで構成されています。

- **Perception:** センサーデータから環境を認識します。
- **Localization:** 自車位置を認識します。
- **Planning:** 自車のパスと動作を計画します。
- **Control:** 計画された動作を実行します。

**コンポーネント**

**Perception**

Perceptionコンポーネントは以下の機能を担当します。

- **カメラ:** 画像データの処理
- **LiDAR:** 3D点群データの処理
- **レーダー:** 物体の検出と速度測定
- **センサーフュージョン:** 複数のセンサーからのデータを統合

**Localization**

Localizationコンポーネントは以下の機能を担当します。

- **自己位置推定:** IMUとGPSデータを使用して自車位置を推定
- **マップマッチング:** 自車位置を地図と照合
- **同時位置合わせとマッピング (SLAM):** 環境をマッピングしながら自車位置を推定

**Planning**

Planningコンポーネントは以下の機能を担当します。

- **経路計画:** 起点から目的地までの経路を計画
- **動作計画:** 経路に沿った自車の動作を計画 (速度、加速度、操舵角)
- **障害回避:** 障害物を回避するための回避策の生成

**Control**

Controlコンポーネントは以下の機能を担当します。

- **ステアリング:** 計画された操舵角の実行
- **ブレーキング:** 計画されたブレーキ圧力の適用
- **アクセル:** 計画されたアクセルペダル位置の実行
- **車両安定化:** 車両の安定性を維持

**連携**

各コンポーネントは、以下のように連携して動作します。

- Perceptionコンポーネントは、環境に関する情報を収集します。
- Localizationコンポーネントは、自車位置を推定します。
- Planningコンポーネントは、自車の経路と動作を計画します。
- Controlコンポーネントは、計画された動作を実行します。

**考慮事項**

自動運転ソフトウェアを設計および実装する際には、以下の考慮事項を考慮する必要があります。

- **精度:** コンポーネントの出力の正確性
- **堅牢性:** コンポーネントのエラーに対する耐性
- **リアルタイム性:** コンポーネントの応答時間
- **計算コスト:** コンポーネントが使用する計算リソース
- **センサーの限界:** センサーの能力と制限

| 名前                                           | 型                                 | 説明                                                              |
| ------------------------------------------------ | ---------------------------------- | ------------------------------------------------------------------- |
| `output/line_segments_cloud`                     | `sensor_msgs::msg::PointCloud2` | ビジュアライゼーション用のフィルタ処理済み線分                     |
| `output/projected_image`                         | `sensor_msgs::msg::Image`       | ビジュアライゼーション用の投影されたフィルタ処理済み線分             |
| `output/projected_line_segments_cloud`           | `sensor_msgs::msg::PointCloud2` | 投影されたフィルタ処理済み線分                                     |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_image_processing/schema/segment_filter.schema.json") }}

## undistort

### 目的

このノードは、画像のリサイズと歪み補正を同時に実行します。

### 入力 / 出力

#### 入力
 - camera_topic(`/{}'.format(camera_name))
   - 型: sensor_msgs/Image
   - 説明: 歪んだカメラ画像

#### 出力
 - camera_topic_undistort
   - 型: sensor_msgs/Image
   - 説明: ゆがみの補正されたカメラ画像

| 名前                                | タイプ                                                | 説明                                              |
| ------------------------------------ | --------------------------------------------------- | --------------------------------------------------- |
| `input/camera_info`                  | `sensor_msgs::msg::CameraInfo`                        | カメラ情報                                          |
| `input/image_raw`                    | `sensor_msgs::msg::Image`                             | 未加工のカメラ画像                                 |
| `input/image_raw/compressed`         | `sensor_msgs::msg::CompressedImage`                   | 圧縮されたカメラ画像                                |

このノードは、圧縮画像と生画像の両方のトピックをサブスクライブします。
生画像が一度でもサブスクライブされると、圧縮画像のサブスクライブが停止します。
これは Autoware 内での不要な解凍を避けるためです。

#### 出力

| 名前                 | タイプリスト                         | 説明                   |
| -------------------- | ------------------------------------ | ------------------------- |
| `output/camera_info` | `sensor_msgs::msg::CameraInfo`      | リサイズされたカメラ情報           |
| `output/image_raw`   | `sensor_msgs::msg::CompressedImage` | 非歪とリサイズされた画像 |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_image_processing/schema/undistort.schema.json", true) }}

#### tf_static オーバーライドについて

<details><summary>開くにはクリック</summary><div>

いくつかのノードは `/base_link` から `/sensing/camera/traffic_light/image_raw/compressed` のフレーム ID (例: `/traffic_light_left_camera/camera_optical_link`) への `/tf_static` を必要とします。tf_static が正しいかどうかは以下のコマンドで確認できます。


```shell
ros2 run tf2_ros tf2_echo base_link traffic_light_left_camera/camera_optical_link
```

プロトタイプ車両の使用、正確なキャリブレーションデータの欠如、またはその他の回避できない理由によって間違った`/tf_static`がブロードキャストされた場合、 `override_camera_frame_id`内の`frame_id`を指定すると便利です。非空の文字列を指定すると、`/image_processing/undistort_node`は`camera_info`内の`frame_id`を書き換えます。たとえば、以下のように異なる`tf_static`を指定できます。


```shell
ros2 launch yabloc_launch sample_launch.xml override_camera_frame_id:=fake_camera_optical_link
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link \
  --child-frame-id fake_camera_optical_link \
  --roll -1.57 \
  --yaw -1.570
```

</div></details>

## lanelet2_overlay

### 目的

このノードは、推定自車位置に基づいて、camera画像にlanelet2を重ね合わせます。

### 入力 / 出力

#### 入力

| 名前                               | タイプ                               | 説明                                           |
| ---------------------------------- | ---------------------------------- | ------------------------------------------------- |
| `input/pose`                         | `geometry_msgs::msg::PoseStamped`  | 自車位置                                          |
| `input/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2`    | 路面マーキング以外の投影された線分                           |
| `input/camera_info`                  | `sensor_msgs::msg::CameraInfo`     | 無歪カメラ情報                                  |
| `input/image_raw`                    | `sensor_msgs::msg::Image`          | 無歪カメラ画像                                  |
| `input/ground`                       | `std_msgs::msg::Float32MultiArray` | 地面勾配                                          |
| `input/ll2_road_marking`             | `sensor_msgs::msg::PointCloud2`    | 路面マーキングに関するlanelet2要素                  |
| `input/ll2_sign_board`               | `sensor_msgs::msg::PointCloud2`    | 交通標識に関するlanelet2要素                          |

**自動運転ソフトウェアに関するドキュメント**

**Planningモジュール**

**概要**

Planningモジュールは、自車位置と周囲環境データを元に、安全かつ効率的な経路計画を行います。以下の機能を備えています。

**機能**

* **経路生成:** 自車位置から目的地までの最適な経路を計算します。
* **速度計画:** 経路に沿った安全かつ快適な速度プロファイルを計算します。
* **衝突回避:** 周囲の障害物との衝突を回避するための回避操作を計算します。
* **車線維持:** 車線を維持するためのステアリング制御を計算します。

**アーキテクチャ**

Planningモジュールは、以下で構成されています。

* **TrajGen:** 経路を生成します。
* **TwistGenerator:** 速度プロファイルを計算します。
* **ObstacleManager:** 障害物を管理します。
* **LaneManager:** 車線を管理します。

**入出力**

**入力:**

* 自車位置
* 周囲環境データ (LiDAR、カメラなど)

**出力:**

* 計画された経路
* 目標速度と加速度
* ステアリング制御

**インターフェイス**

Planningモジュールは、Autoware ROSインタフェースを通じて他のコンポーネントとやり取りします。

**パラメータ**

Planningモジュールのパラメータは、`/autoware/planning`ネームスペースから設定できます。

**パフォーマンス**

Planningモジュールの性能は、以下によって向上できます。

* 高精度のセンサーデータを使用する
* 現在の実速度をフィードバックする
* 障害物の位置を正確に予測する

**制限事項**

* 静的障害物のみを処理します。
* 悪天候では性能が低下する可能性があります。
* 急カーブや狭い道路で問題が発生する可能性があります。

**依存関係**

Planningモジュールには、以下のコンポーネントが必要です。

* PointCloudPreprocessor
* Mapping
* Localization
* Prediction
* ObstacleDetection
* Control

**テスト**

Planningモジュールは、シミュレーションと実車テストの両方でテストされています。

**ドキュメント**

詳細については、以下のドキュメントを参照してください。

* [Planningモジュール設計ドキュメント](https://github.com/autowarefoundation/autoware.ai/blob/master/docs/design/planning-module.md)
* [Planningモジュールソースコード](https://github.com/autowarefoundation/autoware.ai/tree/master/ros/autoware_planning)

**追加情報**

* Planningモジュールでは、`post resampling`などの手法を使用しています。
* Planningモジュールは、velocity逸脱量やacceleration逸脱量などのパラメータを調整できます。
* Planningモジュールは、他のPlanningアルゴリズムと統合できます。

| 名前                              | 型                                | 説明                                                     |
| ------------------------------- | --------------------------------- | ---------------------------------------------------------- |
| `output/lanelet2_overlay_image` | `sensor_msgs::msg::Image`         | レーンレット2のオーバーレイ画像                                |
| `output/projected_marker`       | `visualization_msgs::msg::Marker` | 路面マーキング以外の3D投影された線分                              |

## line_segments_overlay

### 目的

このノードはカメラ画像上に分類された線分を可視化します

### 入出力

#### 入力

| 名前                        | タイプ                            | 説明              |
| --------------------------- | ------------------------------- | ------------------------ |
| `input/line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | 分類線分 |
| `input/image_raw`           | `sensor_msgs::msg::Image`       | 補正解除カメラ画像 |

#### 出力

**自動運転ソフトウェア**

**計画コンポーネント**

計画コンポーネントは、周囲の環境を認識し、それに応じて経路を計画します。

* **地図管理:** ナビゲーションマップのロード、解析、更新を担当。
* **センサーフュージョン:** レーダー、LiDAR、カメラなどのセンサーデータを集約し、周囲の環境の包括的な表現を作成。
* **オドメトリー:** 自車位置の追跡と推定。
* **パスプランニング:** 障害物や交通規制を考慮した安全で効率的な経路の生成。
* **トラフィック予測:** 周辺車両の軌跡を予測し、それに応じて経路を調整。

**制御コンポーネント**

制御コンポーネントは、計画コンポーネントによって生成された経路に従って車両を操作します。

* **縦制御:** 加減速の制御を担当。
* **横制御:** ステアリングの制御を担当。
* **ブレーキ制御:** ブレーキの適用を担当。
* **アクチュエーターインターフェース:** コントローラコマンドと車両アクチュエーターのインターフェース。
* **安定性制御:** 車両の安定性を確保する機能を提供。

**他のコンポーネント**

* **ビヘイビアープランナー:** 車両の一般的な動作を指定。
* **RVIZビジュアライザー:** システムの動作を可視化する。
* **ロギングシステム:** システムの動作とデータを記録。

**Autowareのアーキテクチャ**

* **モジュール性:** 各コンポーネントは独立しており、他のコンポーネントと簡単に統合できます。
* **再利用性:** コンポーネントは再利用可能で、さまざまな自動運転システムで利用できます。
* **オープンソース:** Autowareはオープンソースプラットフォームであり、コミュニティによって継続的に開発されています。

**使用上の注意**

* システムは依然として開発中であり、限定的な使用に限定されています。
* システムを使用する前に、関連する安全対策を講じてください。
* 'post resampling`のデータを使用する場合、元のデータの不確実性と時間遅延に注意してください。
* 速度逸脱量または加速度逸脱量が高い場合、システムの応答が遅れる可能性があります。

| 名前                                                | 型                         | 説明                                                         |
| --------------------------------------------------- | --------------------------- | ------------------------------------------------------------- |
| `output/image_with_colored_line_segments` | `sensor_msgs::msg::Image` | 線分がハイライトされた画像                                  |


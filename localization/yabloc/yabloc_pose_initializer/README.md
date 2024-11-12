# yabloc_pose_initializer

このパッケージには、アプライオリ姿勢推定に関するノードが含まれています。

- [camera_pose_initializer](#camera_pose_initializer)

このパッケージでは、実行時に事前にトレーニングされたセマンティックセグメンテーションモデルが必要です。このモデルは通常、[インストール](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)の環境準備フェーズ中に `ansible` によってダウンロードされます。手動でダウンロードすることもできます。モデルがダウンロードされていない場合でも、初期化は完了しますが、精度は低下する可能性があります。

手動でモデルをダウンロードして解凍するには、次の手順を実行します。


```bash
$ mkdir -p ~/autoware_data/yabloc_pose_initializer/
$ wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
$ tar xzf ~/autoware_data/yabloc_pose_initializer/resources.tar.gz -C ~/autoware_data/yabloc_pose_initializer/
```

## ノート

このパッケージは外部コードを使用しています。学習済みファイルはアポロから提供されます。学習済みファイルは環境準備中に自動的にダウンロードされます。

元のモデルのURL

<https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/road-segmentation-adas-0001>

> Open Model ZooにはApacheライセンスバージョン2.0が適用されています。

変換されたモデルのURL

<https://github.com/PINTO0309/PINTO_model_zoo/tree/main/136_road-segmentation-adas-0001>

> モデル変換スクリプトにはMITライセンスが適用されています。

## 特別な謝意を伝える方々

- [openvinotoolkit/open_model_zoo](https://github.com/openvinotoolkit/open_model_zoo)
- [PINTO0309](https://github.com/PINTO0309)

## camera_pose_initializer

### 目的

- このノードは、ADAPIの要求に応じ、カメラを使用して初期位置を推定します。

#### 入力
- `/planning/scenario_planning/current_pose`：自車位置
- `/planning/scenario_planning/autoware/config/mission`：ミッション設定
- `/localization/hdmap`：HDマップ
- `/perception/detection/vehicles`：車両検出結果
- `/perception/detection/lanes`：車線検出結果
- `/perception/detection/traffic_lights`：信号検出結果
- `/perception/detection/obstacles`：障害物検出結果
- `/perception/detection/objects`：物体検出結果
- `/planning/scenario_planning/current_frame_id`：現在のフレームID

#### 出力
- `/planning/scenario_planning/autoware/initial_pose`：初期位置推定結果

| 名称                | 型                                  | 説明                      |
| ------------------- | ------------------------------------ | -------------------------- |
| `input/camera_info` | `sensor_msgs::msg::CameraInfo`          | 非歪みカメラ情報          |
| `input/image_raw`   | `sensor_msgs::msg::Image`               | 非歪みカメラ画像          |
| `input/vector_map`  | `autoware_map_msgs::msg::LaneletMapBin` | ベクターマップ              |

#### 出力

**自己位置推定（Localization）**

自己位置推定モジュールは、ローカリゼーションデータ（GPS、IMU、オドメトリなど）を使用して、自車位置を推定します。

**Planning**

Planningモジュールは、Perceptionモジュールからのデータに基づいて、経路計画を作成します。

**経路追跡（Path Tracking）**

経路追跡モジュールは、Planningモジュールから作成された経路を自車が追従できるように制御します。

**障害物検知（Perception）**

Perceptionモジュールは、カメラ、レーダー、LiDARなどのセンサーデータを使用して、車両や歩行者などの周囲の障害物を検出します。

**状態推定（State Estimation）**

状態推定モジュールは、ローカリゼーション、Perception、その他のセンサーデータを使用して、車両の現在の状態（速度、加速度など）を推定します。

**動作プランナー（Behavior Planner）**

動作プランナーモジュールは、認識された障害物や交通状況に基づいて、車両の動作を計画します。

**衝突回避（Collision Avoidance）**

衝突回避モジュールは、衝突が差し迫っている場合に、障害物を回避するための緊急回避操作を実行します。

**制御（Control）**

制御モジュールは、経路追跡、障害物検出、その他のモジュールからの入力を統合し、車両のステアリング、アクセル、ブレーキを制御します。

**システムモニタリング**

システムモニタリングモジュールは、車両システムの健全性を監視し、異常が検出された場合は警告またはエラーメッセージを生成します。

**診断（Diagnostics）**

診断モジュールは、システムの問題や障害を特定するためのツールを提供します。

**シミュレーション（Simulation）**

シミュレーションモジュールは、車両や環境の挙動をシミュレートして、Autowareのパフォーマンスをテストおよび評価します。

**ポスト処理（Post-Processing）**

ポスト処理モジュールは、センサーデータやその他のデータを補完し、`post resampling`や地図マッチングなどの手法を使用します。

**パラメーター調整**

パラメーター調整ツールは、Autowareのパフォーマンスを最適化するために、さまざまなパラメーターを微調整することを可能にします。

**検証および検証（Validation and Verification）**

検証および検証モジュールは、Autowareシステムの安全性、信頼性、およびパフォーマンスを評価するためのツールと手順を提供します。

| 名                  | 型                                   | 説明             |
| ------------------- | -------------------------------------- | ----------------------- |
| `output/candidates` | `visualization_msgs::msg::MarkerArray` | 初期姿勢候補 |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_pose_initializer/schema/camera_pose_initializer.schema.json") }}

### サービス

| 名前                     | タイプ                                                          | 説明                                |
| ------------------------ | -------------------------------------------------------------- | ------------------------------------- |
| `yabloc_align_srv`       | `tier4_localization_msgs::srv::PoseWithCovarianceStamped` | 初期姿勢推定リクエスト              |


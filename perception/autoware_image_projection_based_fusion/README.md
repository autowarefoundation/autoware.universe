## autoware_image_projection_based_fusion

## 目的

`autoware_image_projection_based_fusion`は、イメージと3Dポイントクラウドまたは障害物（バウンディングボックス、クラスタ、またはセグメンテーション）から検出された障害物（バウンディングボックスまたはセグメンテーション）をフュージョンするためのパッケージです。

## 処理内容 / アルゴリズム

### 同期アルゴリズム

#### マッチング

各カメラとLiDARのオフセットはそのシャッタータイミングによって設定されています。
タイムスタンプにオフセットを適用した後、ポイントクラウドトピックのタイムスタンプとROIメッセージ間のインターバルがマッチングの閾値よりも小さい場合、2つのメッセージはマッチングされます。

![roi_sync_image1](./docs/images/roi_sync_1.png)

Autoware.universeのTIER IV Robotaxiでの現在のデフォルト値は次のとおりです。
- input_offset_ms: [61.67、111.67、45.0、28.33、78.33、95.0]
- match_threshold_ms: 30.0

#### フュージョンとタイマー

![roi_sync_image2](./docs/images/roi_sync_2.png)

メッセージのサブスクリプションステータスは「O」で示されます。

1. あるポイントクラウドメッセージが以下の条件を満たしてサブスクライブされる場合、

(a). a camera image is matched
(b). a timer starts counting to the pre-configured timeout value.

2. if a camera image is matched, the timer resets.

3. if the timer expires, the sign is reset to 'X' and the subscription will be terminated.

### Communication

#### Sensor independence

The fusion module can works only with pointcloud data if image data is not available, or only with image data if pointcloud data is not available.

#### No need to share coordinate frame

The fusion module works on `post resampling` pointcloud and Bird's-eye-view (BEV) camera image.
The fusion module does not have to share the coordinate frame with the sensors, because each sensor has its own T(leader -> subject) transform.

#### Sensor timing management

Sensor timing is managed by the ROI sync module.
It is required for each Planning sensor to have its own topics for ROI and data.

### Integration with Planning

The fusion module has no strong dependency on the Planning module.
The Planning module can get all the necessary information from the fused messages.

## Installation

source code directory: `autoware/autoware/core/autoware_image_projection_based_fusion`

## Usage

### Required stack

- ROS
- autoware\_can\_msgs
- autoware\_perception\_msgs
- autoware\_planning\_msgs
- cv\_bridge
- octomap\_ros

### Example

tum\_slayer\_1/base\_linkへ変換されたcamera1のCameraInfoと、fusion\_imageのサブスクライバを起動します。

``` bash
roslaunch autoware_image_projection_based_fusion fusion.launch image_topic:=/camera1/front_left/compressed image_info_frame_id:=/base_link output_topic:=/fusion_image fusion_node_name:=fused_image
```

|                     | pointcloud | roi msg 1 | roi msg 2 | roi msg 3 |
| :-----------------: | :--------: | :-------: | :-------: | :-------: |
| サブスクリプションステータス |            |     有     |     有     |     有     |

roi msgsにマッチング可能な場合はそれらを融合してポイントクラウドメッセージを処理します。
それ以外の場合はマッチングしたroi msgsを融合してポイントクラウドをキャッシュします。

2.次の条件下でポイントクラウドメッセージにサブスクライブした場合：

|                     | pointcloud | roi msg 1 | roi msg 2 | roi msg 3 |
| :-----------------: | :--------: | :-------: | :-------: | :-------: |
| 受信状態             |            |     O     |     O     |           |

1. ROI メッセージが照合できる場合、それらを融合し、点群をキャッシュします。

2. 以下条件下で点群メッセージが購読された場合:

|                   | pointcloud | roi msg 1 | roi msg 2 | roi msg 3 |
| :----------------: | :--------: | :-------: | :-------: | :-------: |
| サブスクリプション状態 |     ○      |     ○     |     ○     |

roi msg 3 が次のポイントクラウドメッセージの受信またはタイムアウト前にサブスクライブされている場合は、一致した場合に融合し、そうでなければ次の roi msg 3 を待ちます。

roi msg 3 が次のポイントクラウドメッセージの受信またはタイムアウト前にサブスクライブされていない場合は、そのままポイントクラウドメッセージを事後処理します。

タイムアウトのしきい値は、事後処理時間に応じて設定する必要があります。
たとえば、事後処理時間が約50ミリ秒の場合、タイムアウトのしきい値は50ミリ秒未満に設定する必要があり、全体の処理時間が100ミリ秒未満になるようにする必要があります。
Autoware.universe での現在のデフォルト値：XX1: - timeout_ms: 50.0

#### `build_only` オプション

`pointpainting_fusion` ノードには、ONNX ファイルから TensorRT エンジンファイルを構築するための `build_only` オプションがあります。
Autoware Universe の `.param.yaml` ファイルのすべての ROS パラメータを移動させることが好まれますが、`build_only` オプションはプレタスクとしてビルドを実行するためのフラグとして使用される可能性があるため、今のところ `.param.yaml` ファイルには移動されていません。次のコマンドで実行できます。


```bash
ros2 launch autoware_image_projection_based_fusion pointpainting_fusion.launch.xml model_name:=pointpainting model_path:=/home/autoware/autoware_data/image_projection_based_fusion model_param_path:=$(ros2 pkg prefix autoware_image_projection_based_fusion --share)/config/pointpainting.param.yaml build_only:=true
```

#### 制限事項

rclcpp::TimerBase таймерは for ループを break できないため、roi メッセージを中間に融合するときに時間が切れた場合でも、すべてのメッセージが融合されるまでプログラムは実行されます。

### 各融合アルゴリズムの詳細な説明は次のリンクにあります

| フュージョン名 | 説明 | 詳細 |
|---|---|---|
| `roi_cluster_fusion` | 2Dオブジェクト検出器のROIから、クラスタの分類ラベルを上書き | [リンク](./docs/roi-cluster-fusion.md) |
| `roi_detected_object_fusion` | 2Dオブジェクト検出器のROIから、検出オブジェクトの分類ラベルを上書き | [リンク](./docs/roi-detected-object-fusion.md) |
| `pointpainting_fusion` | 2Dオブジェクト検出器のROIで点群にペイントし、3Dオブジェクト検出器にフィード | [リンク](./docs/pointpainting-fusion.md) |
| `roi_pointcloud_fusion` | 2Dオブジェクト検出器のROIと点群を照合し、ラベル不明のオブジェクトを検出 | [リンク](./docs/roi-pointcloud-fusion.md) |


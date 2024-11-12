# Reaction Analyzer

## 説明

reaction analyzerパッケージの主な目的は、ROSベースの自動運転シミュレーション環境内のさまざまなノードの反応時間を、事前に決定されたトピックをサブスクライブすることで測定することです。このツールは特に、急な障害物などの環境の変化に対する知覚、Plannining、制御パイプラインのパフォーマンスを評価するのに役立ちます。制御出力と知覚出力を両方を測定できるように、ノードを2つのrunning_mode（`planning_control`と`perception_planning`）に分割する必要がありました。

![ReactionAnalyzerDesign.png](media%2FReactionAnalyzerDesign.png)

### Plannining制御モード

このモードでは、reaction analyzerは、PredictedObjectsトピックとPointCloud2トピックのダミーパブリッシャーを作成します。テストの開始時に、自己位置の初期値と目標値を公開して、テスト環境を設定します。次に、自己車両の前に突然の障害物を産出します。障害物が発生すると、事前に決定されたトピックで、Planniningノードと制御ノードの応答メッセージの検索が開始されます。すべてのトピックが反応すると、各ノードの`spawn_cmd_time`に対する`reacted_times`を比較して、ノードの反応時間と統計を計算し、結果を格納するCSVファイルを作成します。

### 知覚Planniningモード

このモードでは、reaction analyzerはAWSIMから記録されたrosbagファイルを読み取り、rosbagを再生するためにrosbag内の各トピックのトピックパブリッシャーを作成します。`path_bag_without_object`と`path_bag_with_object`の2つのrosbagファイルが読み取られます。最初に、`path_bag_without_object`を再生して自己車両の初期位置と目標位置を設定します。`spawn_time_after_init`秒後、`path_bag_with_object`を再生して、自己車両の前に突然の障害物を産出します。障害物が発生すると、事前に決定されたトピックで、知覚ノードとPlanniningノードの応答メッセージの検索が開始されます。すべてのトピックが反応すると、各ノードの`spawn_cmd_time`に対する`reacted_times`を比較して、ノードの反応時間と統計を計算し、結果を格納するCSVファイルを作成します。

#### Point Cloud Publisherのタイプ

知覚・センシングパイプラインのより優れた分析を得るために、reaction analyzerは3つの異なる方法（`async_header_sync_publish`、`sync_header_sync_publish`、または`async_publish`）でポイントクラウドメッセージを公開できます。（`T`はlidarの出力の周期です）

![PointcloudPublisherType.png](media%2FPointcloudPublisherType.png)

- `async_header_sync_publish`：非同期ヘッダ時間と同期してポイントクラウドメッセージを公開します。つまり、lidarの出力はすべて同じ時間に公開されますが、ポイントクラウドメッセージのヘッダには位相差のために異なるタイムスタンプが含まれます。
- `sync_header_sync_publish`：同期ヘッダ時間と同期してポイントクラウドメッセージを公開します。つまり、lidarの出力はすべて同じ時間に公開され、ポイントクラウドメッセージのヘッダには同じタイムスタンプが含まれます。
- `async_publish`：非同期にポイントクラウドメッセージを公開します。つまり、lidarの出力はすべて異なる時間に公開されます。

## 使用法

どちらのランニングモードでも定義する必要がある一般的なパラメータは、`output_file_path`、`test_iteration`、および`reaction_chain`リストです。`output_file_path`は、結果と統計が格納される出力ファイルのパスです。`test_iteration`は、実行されるテストの数を定義します。`reaction_chain`リストは、反応時間を測定する事前に定義されたトピックのリストです。

**重要:** `reaction_chain`リストが正しく定義されていることを確認してください。

- `perception_planning`モードでは、`Control`ノードを定義しないでください。

### 事前に準備されたテスト環境

- デモテスト用マップは、[こちら](https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip)のリンクからダウンロードできます。ダウンロード後、zipファイルを展開して、パスを以下のコマンドの`[MAP_PATH]`として使用します。

#### 計画制御モード

- 次の`reaction_chain`リストで、計画ノードと制御ノードのみを定義する必要があります。デフォルトのパラメータで、次のコマンドでテストを開始できます。


```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=planning_control vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit map_path:=[MAP_PATH]
```

コマンド実行後、`simple_planning_simulator` と `reaction_analyzer` が起動されます。テストが自動的に開始されます。テストの終了後、結果は定義した `output_file_path` に保存されます。

#### 知覚プランニングモード

- Google Drive から rosbag ファイルをダウンロードします。
  リンクは [こちら](https://drive.google.com/drive/folders/1eJMEdt4WbU-W6MPXlNTkIhZtwpof0HcO?usp=sharing)。
- zip ファイルを展開し、`.db3` ファイルのパスをパラメーター `path_bag_without_object` と `path_bag_with_object` に設定します。
- 以下のコマンドでテストを開始できます。


```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=perception_planning vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=[MAP_PATH]
```

- 最初に perception_planning モードでツールを実行する際、初期化が予想より長引く場合があります。処理が完了するまでしばらくお待ちください。

コマンドを実行すると、 `e2e_simulator` と `reaction_analyzer` が起動します。自動的にテストが開始されます。テストが完了すると、定義した `output_file_path` に結果が保存されます。

#### 準備されたテスト環境

**障害物なしのシーン:**
![sc1-awsim.png](media%2Fsc1-awsim.png)
![sc1-rviz.png](media%2Fsc1-rviz.png)

**障害物のあるシーン:**
![sc2-awsim.png](media%2Fsc2-awsim.png)
![sc2-rviz.png](media%2Fsc2-rviz.png)

### カスタムテスト環境

**カスタムテスト環境でリアクションアナライザを実行する場合は、パラメータの一部を再定義する必要があります。
再定義する必要があるパラメータは `initialization_pose`, `entity_params`, `goal_pose`, `topic_publisher` （`perception_planning` モード用）パラメータです。**

- `initialization_pose`, `entity_params`, `goal_pose` を設定するには:
- AWSIM 環境を実行します。AWSIM のチュートリアルは [こちら](https://autowarefoundation.github.io/AWSIM/main/GettingStarted/QuickStartDemo/) からご覧いただけます。
- 次のコマンドで e2e_simulator を実行します。


```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=[MAP_PATH]
```

- EGO が初期化された後、RViz の `2D Pose Estimate` ボタンを使用して、自車を希望の位置に配置できます。
- 自車が希望の位置に配置された後、交通管制を利用してダミー障害物を配置してください。交通管制セクションには Esc キーを押してアクセスできます。

**EGO とダミー車両を配置したら、これらのエンティティの位置を `reaction_analyzer.param.yaml` 内の地図フレームに書き込む必要があります。この方法：**

- `/awsim/ground_truth/vehicle/pose` トピックから初期化ポーズを取得します。
- `/perception/object_recognition/objects` トピックからエンティティのパラメータを取得します。
- `/planning/mission_planning/goal` トピックから目標ポーズを取得します。

**注：`initialization_pose` は `planning_control` モードでのみ有効です。**

- パラメータが記録された後、テスト用の ROS バッグを記録する必要があります。ROS バッグを記録するには、次のコマンドを使用できます。


```bash
ros2 bag record --all
```

- オブジェクトがない状態とオブジェクトがある状態の 2 つの rosbag を記録する必要があります。トラフィックコントローラーを使用して、EGO ビークルの前にオブジェクトを生成するか削除できます。

**注: 記録時に同じ環境で EGO ビークルの同じ位置を使用する必要があります。記録中に Autoware を実行する必要はありません。**

- Rosbag を記録したら、`path_bag_without_object` パラメータと `path_bag_with_object` パラメータに、記録された rosbag のパスを設定できます。

## 結果

結果は `csv` ファイル形式で保存され、定義した `output_file_path` に書き込まれます。メッセージのヘッダータイムスタンプを使用して Autoware の各パイプラインを表示し、各ノードの `ノード遅延`、`パイプライン遅延`、および `合計遅延` をレポートします。

- `ノード遅延`: 直前のノードの反応タイムスタンプと現在のノードの反応タイムスタンプの時差です。パイプライン内の最初のノードの場合、`パイプライン遅延` と同じです。
- `パイプライン遅延`: メッセージの公開時間とパイプラインヘッダー時間の時差です。
- `合計遅延`: メッセージの公開タイムスタンプと障害物の生成コマンドが送信されたタイムスタンプの時差です。

## パラメータ

| 名称                                                                          | タイプ   | 説明                                                                                                                                        |
| ----------------------------------------------------------------------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `timer_period`                                                                | double | [s] メイン処理タイマーのピリオド。                                                                                                                |
| `test_iteration`                                                              | int    | テストの繰り返し回数。                                                                                                                          |
| `output_file_path`                                                           | string | テスト結果と統計情報を格納するディレクトリパス。                                                                                              |
| `spawn_time_after_init`                                                      | double | [s] オブジェクトをスポーンする前の初期化後のタイムディレイ。 `perception_planning` モードでのみ有効。                                         |
| `spawn_distance_threshold`                                                   | double | [m] オブジェクトスポーンの距離しきい値。 `planning_control` モードでのみ有効。                                                               |
| `poses.initialization_pose`                                                  | struct | 車両の初期姿勢で `x`, `y`, `z`, `roll`, `pitch`, `yaw` フィールドを含む。 `planning_control` モードでのみ有効。                            |
| `poses.entity_params`                                                        | struct | エンティティ（障害物など）のパラメータで `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `x_dimension`, `y_dimension`, `z_dimension` を含む。 |
| `poses.goal_pose`                                                            | struct | 車両のゴール姿勢で `x`, `y`, `z`, `roll`, `pitch`, `yaw` フィールドを含む。                                                              |
| `topic_publisher.path_bag_without_object`                                    | string | オブジェクトなしの ROS バッグファイルへのパス。 `perception_planning` モードでのみ有効。                                                  |
| `topic_publisher.path_bag_with_object`                                       | string | オブジェクトありの ROS バッグファイルへのパス。 `perception_planning` モードでのみ有効。                                                   |
| `topic_publisher.spawned_pointcloud_sampling_distance`                       | double | [m] スポーンされたオブジェクトの点群のサンプリング距離。 `planning_control` モードでのみ有効。                                             |
| `topic_publisher.dummy_perception_publisher_period`                          | double | [s] ダミー知覚データの公開周期。 `planning_control` モードのみ有効。                                                                     |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_type`             | string | PointCloud2 メッセージの公開方法を定義します。上記で説明したモード。                                                                       |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_period`           | double | [s] PointCloud2 メッセージの公開周期。                                                                                             |
| `topic_publisher.pointcloud_publisher.publish_only_pointcloud_with_object`   | bool   | デフォルトは false。オブジェクトのある点群メッセージのみを公開します。                                                                     |
| `reaction_params.first_brake_params.debug_control_commands`                  | bool   | デバッグ公開フラグ。                                                                                                                        |
| `reaction_params.first_brake_params.control_cmd_buffer_time_interval`        | double | [s] 制御コマンドのバッファー処理のタイムインターバル。                                                                                   |
| `reaction_params.first_brake_params.min_number_descending_order_control_cmd` | int    | ブレーキトリガーの降順制御コマンドの最小数。                                                                                             |
| `reaction_params.first_brake_params.min_jerk_for_brake_cmd`                  | double | [m/s³] ブレーキコマンドを発行するための最小ジャーク値。                                                                                      |
| `reaction_params.search_zero_vel_params.max_looking_distance`                | double | [m] 軌道上でゼロ速度を探すための最大探索距離。                                                                                             |
| `reaction_params.search_entity_params.search_radius`                         | double | [m] スポーンされたエンティティの検索半径。自己位置とエンティティ位置間の距離。                                                               |
| `reaction_chain`                                                             | struct | トピックとそのトピックのメッセージタイプを持つノードのリスト。                                                                               |

## 制限

- 反応解析器は、`PublisherMessageType`、`SubscriberMessageType`、`ReactionType` などの制限があります。現時点では以下のリストをサポートしています。

- **Publisher メッセージ タイプ:**

  - `sensor_msgs/msg/PointCloud2`
  - `sensor_msgs/msg/CameraInfo`
  - `sensor_msgs/msg/Image`
  - `geometry_msgs/msg/PoseWithCovarianceStamped`
  - `sensor_msgs/msg/Imu`
  - `autoware_vehicle_msgs/msg/ControlModeReport`
  - `autoware_vehicle_msgs/msg/GearReport`
  - `autoware_vehicle_msgs/msg/HazardLightsReport`
  - `autoware_vehicle_msgs/msg/SteeringReport`
  - `autoware_vehicle_msgs/msg/TurnIndicatorsReport`
  - `autoware_vehicle_msgs/msg/VelocityReport`

- **Subscriber メッセージ タイプ:**

  - `sensor_msgs/msg/PointCloud2`
  - `autoware_perception_msgs/msg/DetectedObjects`
  - `autoware_perception_msgs/msg/TrackedObjects`
  - `autoware_perception_msgs/msg/PredictedObject`
  - `autoware_planning_msgs/msg/Trajectory`
  - `autoware_control_msgs/msg/Control`

- **反応のタイプ:**
  - `FIRST_BRAKE`
  - `SEARCH_ZERO_VEL`
  - `SEARCH_ENTITY`

## 将来の改善点

- 反応分析は、より多くの反応タイプを追加することで改善できます。現在サポートしているのは `FIRST_BRAKE`、`SEARCH_ZERO_VEL`、`SEARCH_ENTITY` だけです。メッセージ タイプごとに反応タイプを追加することで拡張できます。


# autoware_carla_interface

## ROS 2/Autoware.universe bridge for CARLA simulator

<https://github.com/gezp>によるROS 2 Humble対応のCARLA Communicationに感謝いたします。
このROSパッケージにより、自律走行シミュレーションのためのAutowareとCARLA間の通信が可能になります。

## サポートされている環境

| ubuntu |  ros   | carla  | autoware |
| :----: | :----: | :----: | :------: |
| 22.04  | humble | 0.9.15 |   Main   |

## 自動運転ソフトウェアドキュメント

### Planningコンポーネント

#### Sensingモジュール

このモジュールは、センサーデータから車線を検出してトラッキングします。

#### Planningモジュール

このモジュールは、以下に基づいて、自車位置と目標地点を考慮した、安全な経路を生成します。

- 車両の速度と加速度
- 周囲の環境
- 交通規則

#### Controlモジュール

このモジュールは、Planningモジュールから生成された経路を元に、ステアリング、アクセル、ブレーキを制御します。

### レイヤーのインターフェース

各レイヤーは、以下のようなインターフェースを介して相互に通信します。

- **Sensing**
  - `current pose`
  - センサーデータ
- **Planning**
  - `current pose`
  - 経路
- **Control**
  - 経路
  - `post resampling`車両状態

### エラー処理

以下のようなエラーが発生した場合、次の対応が行われます。

- **センサーの障害**
  - エラーが検出されると、Planningモジュールは経路生成を停止します。
- **Planningの障害**
  - 経路逸脱量の逸脱量が許容値を超えると、Controlモジュールは車両を停止します。
- **Controlの障害**
  - アクセルまたはブレーキの逸脱量が許容値を超えると、Planningモジュールは経路を再計算します。

### パラメータのチューニング

次のパラメータは、ソフトウェアの動作を調整するためにチューニングできます。

- **Sensing**
  - 車線検出手法の閾値
- **Planning**
  - 安全マージンの大きさ
  - 目標速度
- **Control**
  - ステアリングゲイン
  - 加速および減速率

## 設定

### インストール

- [CARLA インストール](https://carla.readthedocs.io/en/latest/start_quickstart/)
- [Carla Lanelet2 マップ](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)
- [CARLA 0.9.15 ROS 2 Humble コミュニケーション用 Python パッケージ](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04)

  - pip を使用して wheel をインストールします。
  - または egg ファイルを `PYTHONPATH` に追加します。

1. マップを任意の場所にダウンロード（y 軸反転バージョン）
2. 名前を変更し、`autoware_map` 内にマップフォルダーを作成（例：Town01）。（`point_cloud/Town01.pcd` -> `autoware_map/Town01/pointcloud_map.pcd`、`vector_maps/lanelet2/Town01.osm`-> `autoware_map/Town01/lanelet2_map.osm`）
3. フォルダーに `map_projector_info.yaml` を作成し、最初の行に `projector_type: local` を追加。

### ビルド


```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 実行

1. carlaを実行し、マップを変更し、必要に応じてオブジェクトを生成する


   ```bash
   cd CARLA
   ./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
   ```

## 2. rosノードの実行

```bash
$ roslaunch autoware ai_node.launch
```

`Autoware`を起動すると、以下のノードが実行されます。
- `Planning` コンポーネント
- `localization` コンポーネント
- `visualization` コンポーネント
- `util` コンポーネント

`current pose` は、`planning` コンポーネントによって自動生成されます。
`current pose` の受信には以下のトピックを使用します。

```
/current_pose
```

`planning` コンポーネントは、`current pose` を元に経路を計画します。
`planning` コンポーネントが計画した経路は、以下のトピックで受信できます。

```
/planned_path
```

`visualization` コンポーネントは、`planning` コンポーネントが計画した経路を視覚化します。

```
/planned_path
```

`visualization` コンポーネントは、以下のトピックで自車位置を受信します。

```
/current_pose
```

`util` コンポーネントは、`post resampling` された経路を出力します。
`post resampling` された経路は、以下のトピックで受信できます。

```
/resampled_path
```

`util` コンポーネントは、以下のトピックで経路の逸脱量を出力します。

```
/path_offset
```


   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/Town01 vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit simulator_type:=carla carla_map:=Town01
   ```

### 内部メカニズム / アルゴリズム

`InitializeInterface`クラスは、CARLAワールドと自動運転車両の両方を設定するために重要です。このクラスは、`autoware_carla_interface.launch.xml`を通じて設定パラメータを取得します。

メインのシミュレーションループは`carla_ros2_interface`クラス内で実行されます。このループは、CARLAシミュレータ内のシミュレーション時間を`fixed_delta_seconds`で刻み、データを受信してROS 2メッセージとして`self.sensor_frequencies`で定義された周波数で公開します。

Autowareからの自動運転車の指令は、`autoware_raw_vehicle_cmd_converter`を通じて処理され、これらの指令はCARLAに合わせて調整されます。調整された指令はその後、`CarlaDataProvider`を介してCARLA制御に直接フィードされます。

### ワールドロード用の設定可能なパラメータ

すべての主要パラメータは、`autoware_carla_interface.launch.xml`で設定できます。

| 名称                       | タイプ | デフォルト値                                                                                  | 説明                                                                                                                                                                                                                  |
| -------------------------- | ------ | ------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `host`                     | 文字列 | "localhost"                                                                                    | CARLA サーバーのホスト名                                                                                                                                                                                                  |
| `port`                     | 整数   | "2000"                                                                                       | CARLA サーバーのポート番号                                                                                                                                                                                             |
| `timeout`                  | 整数   | 20                                                                                           | CARLA クライアントのタイムアウト                                                                                                                                                                                    |
| `ego_vehicle_role_name`    | 文字列 | "ego_vehicle"                                                                                  | 自車ロールの名称                                                                                                                                                                                                |
| `vehicle_type`             | 文字列 | "vehicle.toyota.prius"                                                                           | スポーンさせる車両のブループリント ID。車両のブループリント ID は [CARLA ブループリント ID](https://carla.readthedocs.io/en/latest/catalogue_vehicles/) で見つけることができる                       |
| `spawn_point`              | 文字列 | なし                                                                                        | 自車をスポーンさせる座標（なしの場合はランダム）。形式 = [x, y, z, roll, pitch, yaw]                                                                                                                                     |
| `carla_map`                | 文字列 | "Town01"                                                                                     | CARLA にロードするマップの名称                                                                                                                                                                                    |
| `sync_mode`                | ブール | True                                                                                           | CARLA に同期モードを設定するためのブールフラグ                                                                                                                                                                       |
| `fixed_delta_seconds`      | double | 0.05                                                                                           | シミュレーションのタイムステップ（クライアント FPS に関連付けられている）                                                                                                                                             |
| `objects_definition_file`  | 文字列 | "$(find-pkg-share autoware_carla_interface)/objects.json"                                     | CARLA でセンサーをスポーンするために使用されるセンサーパラメーターファイル                                                                                                                                        |
| `use_traffic_manager`      | ブール | True                                                                                           | CARLA にトラフィックマネージャーを設定するためのブールフラグ                                                                                                                                                                   |
| `max_real_delta_seconds`   | double | 0.05                                                                                           | `fixed_delta_seconds` よりもシミュレーション速度を制限するためのパラメーター                                                                                                                                             |
| `config_file`              | 文字列 | "$(find-pkg-share autoware_carla_interface)/raw_vehicle_cmd_converter.param.yaml"           | `autoware_raw_vehicle_cmd_converter` で使用される制御マッピングファイル。現在の制御は CARLA の `vehicle.toyota.prius` ブループリント ID に基づいてキャリブレーションされている。車両タイプを変更する場合は再キャリブレーションが必要になる可能性がある |

### センサーの構成可能なパラメータ

以下のパラメータは `carla_ros.py` で構成できます。

| 名称                      | タイプ | デフォルト値                                                                          | 説明                                                                                                                                                                                                                       |
| ------------------------- | ---- | -------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `self.sensor_frequencies` | dict | {"top": 11, "left": 11, "right": 11, "camera": 11, "imu": 50, "status": 50, "pose": 2} | (line 67) 前回の公開以降の時間間隔を計算し、この間隔が、所定の頻度を超過しないために必要な最小限度を満たしているかどうかを確認します。これは ROS の公開頻度のみ影響し、CARLA のセンサーの更新には影響しません。 |

- CARLAセンサーパラメータは`config/objects.json`で設定できます。
  - CARLAで設定可能なパラメータの詳細については、[Carla Ref Sensor](https://carla.readthedocs.io/en/latest/ref_sensors/)で説明されています。

### ワールドのロード

`carla_ros.py` はCARLAワールドを設定します。

1. **クライアント接続**:


   ```python
   client = carla.Client(self.local_host, self.port)
   client.set_timeout(self.timeout)
   ```

2. **マップのロード**:

   `carla_map`パラメータに従って、マップをCARLAワールドにロード。


   ```python
   client.load_world(self.map_name)
   self.world = client.get_world()
   ```

3. **自車生成**:

   車両は `vehicle_type`, `spawn_point`, `agent_role_name` のパラメーターに従って生成されます。


   ```python
   spawn_point = carla.Transform()
   point_items = self.spawn_point.split(",")
   if len(point_items) == 6:
      spawn_point.location.x = float(point_items[0])
      spawn_point.location.y = float(point_items[1])
      spawn_point.location.z = float(point_items[2]) + 2
      spawn_point.rotation.roll = float(point_items[3])
      spawn_point.rotation.pitch = float(point_items[4])
      spawn_point.rotation.yaw = float(point_items[5])
   CarlaDataProvider.request_new_actor(self.vehicle_type, spawn_point, self.agent_role_name)
   ```

## 信号認識

Carla シミュレータ ([Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)) によって提供される地図は現在、Autoware に対する適切な信号コンポーネントが欠けており、点群地図と比較して緯度と経度の座標が異なります。信号認識を可能にするには、以下の手順に従って地図を修正します。

- 地図を修正するオプション

  - A. ゼロから新しい地図を作成する
  - [Tier4 Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) を使用して、新しい地図を作成します。

  - B. 既存の Carla Lanelet2 Maps を修正する
  - PCD (原点) と整列するよう、[Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/) の経度と緯度を調整します。
    - 座標を変更するにはこの [ツール](https://github.com/mraditya01/offset_lanelet2/tree/main) を使用します。
    - Lanelet を PCD とスナップし、[Tier4 Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) を使用して信号を追加します。

- Tier4 Vector Map Builder を使用する場合、PCD フォーマットを `binary_compressed` から `ascii` に変換する必要があります。この変換には `pcl_tools` を使用できます。
- 参考として、1 つの交差点に信号を追加した Town01 の例は [こちら](https://drive.google.com/drive/folders/1QFU0p3C8NW71sT5wwdnCKXoZFQJzXfTG?usp=sharing) からダウンロードできます。

## ヒント

- 初期化中に位置ずれが発生することがありますが、`init by gnss` ボタンを押せば修正できます。
- `fixed_delta_seconds` を変更するとシミュレーションのティックが増加します (デフォルト 0.05 秒)。変更すると、`objects.json` の一部のセンサーのパラメータを調整する必要があります (例: LIDAR 回転周波数は FPS と一致する必要があります)。

## 既知の問題と今後の作業

- 手続き型マップ (Adv Digital Twin) のテストを行います。
  - 現在、Adv Digital Twin map の作成に失敗するためテストできません。
- Autoware センサーキットから CARLA センサーを自動的に設定します。
  - センサーは現在、Autoware センサーキットと同じ場所に配置されるように自動的に設定されていません。現在の回避策は、各センサーの新しいフレームを (0, 0, 0, 0, 0, 0) 座標 (base_link に相対的) で作成し、各センサーを新しいフレームにアタッチすることです (`autoware_carla_interface.launch.xml` 行 28)。この回避策は非常に限定的で制約があります。センサーキットが変更されると、センサーの場所は正しくアタッチされません。
- 信号認識。
  - 現在、CARLA の HDmap には、Autoware が信号認識を行うために必要な信号に関する情報がありません。


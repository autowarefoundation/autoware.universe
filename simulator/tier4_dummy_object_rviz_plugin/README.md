# tier4_dummy_object_rviz_plugin

## 目的

このプラグインは、プランニングシミュレータ内のダミー歩行者、車、障害物を生成するために使用されます。

## 概要

CarInitialPoseToolはダミー車の生成トピックを送信します。  
PedestrianInitialPoseToolはダミー歩行者の生成トピックを送信します。  
UnknownInitialPoseToolはダミー障害物の生成トピックを送信します。  
DeleteAllObjectsToolは上記3つのツールで表示されたダミー車、歩行者、障害物を削除します。

## 入出力

### 出力

| 名前                                                  | タイプ                                       | 説明                                            |
| ---------------------------------------------------- | ------------------------------------------ | ------------------------------------------------ |
| `/simulation/dummy_perception_publisher/object_info` | `tier4_simulation_msgs::msg::DummyObject` | ダミーオブジェクト情報をパブリッシュするトピック |

## パラメータ

### コアパラメータ

#### CarPose
- 定義: 自車位置に関するパラメータ
- 説明: 自車位置の初期値に関するパラメータです。
  例:
  ```
  - car_pose:
      x: 10.0
      y: 20.0
      z: 30.0
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
  ```

#### PlannerGoal

- 定義: プランナーの目標に関するパラメータ
- 説明: プランナーのゴールに関するパラメータです。
  例:
  ```
  - planner_goal:
      x: 40.0
      y: 50.0
      z: 60.0
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
      v: 10.0
      a: 0.0
  ```

#### Trajectory

- 定義: 軌道に関するパラメータ
- 説明: 車両が走行する予定の軌道の初期値に関するパラメータです。
  例:
  ```
  - trajectory:
      x: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
      y: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
      z: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
      v: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
      a: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
  ```

#### BehaviorPlanner

- 定義: 行動プランナーに関するパラメータ
- 説明: 行動プランナーに関するパラメータです。
  例:
  ```
  - behavior_planner:
      target_vel: 10.0
      target_acc: 0.0
      velocity_violation: 0.1
      acceleration_violation: 0.1
  ```

### Perception Parameters

#### Radar

- 定義: レーダーに関するパラメータ
- 説明: レーダーセンサーに関するパラメータです。
  例:
  ```
  - radar:
      fov: 90.0
      range: 100.0
      min_detection_size: 0.1
  ```

#### Lidar

- 定義: LiDARに関するパラメータ
- 説明: LiDARセンサーに関するパラメータです。
  例:
  ```
  - lidar:
      fov: 180.0
      range: 100.0
      min_detection_size: 0.1
  ```

#### Camera

- 定義: カメラに関するパラメータ
- 説明: カメラセンサーに関するパラメータです。
  例:
  ```
  - camera:
      fov: 120.0
      range: 100.0
      min_detection_size: 0.1
  ```

### Control Parameters

#### MPC

- 定義: MPCに関するパラメータ
- 説明: MPCに関するパラメータです。
  例:
  ```
  - mpc:
      dt: 0.01
      horizon: 10
      Q: [1.0, 0.0, 0.0]
      R: [1.0, 0.0, 0.0]
  ```

#### PurePursuit

- 定義: PurePursuitに関するパラメータ
- 説明: PurePursuitに関するパラメータです。
  例:
  ```
  - pure_pursuit:
      lookahead_distance: 10.0
      wheelbase: 2.0
  ```

### Simulation Parameters

#### Simulator

- 定義: シミュレーターに関するパラメータ
- 説明: シミュレーターに関するパラメータです。
  例:
  ```
  - simulator:
      dt: 0.01
      environment: 'urban'
      spawn_location: [10.0, 20.0, 30.0]
      scenario: 'intersection'
  ```

### Autoware

- 定義: Autowareに関するパラメータ
- 説明: Autowareに関するパラメータです。
  例:
  ```
  - autoware:
      debug: false
      verbose: false
      'post resampling': false
  ```

| 名前              | 型   | 初期値                                       | 説明                                   |
| ----------------- | ------ | --------------------------------------------- | -------------------------------------- |
| `topic_property_` | string | `/simulation/dummy_perception_publisher/object_info` | ダミーオブジェクト情報をパブリッシュするトピック |
| `std_dev_x_`      | float  | 0.03                                            | 初期ポーズの X 標準偏差 [m]   |
| `std_dev_y_`      | float  | 0.03                                            | 初期ポーズの Y 標準偏差 [m]   |
| `std_dev_z_`      | float  | 0.03                                            | 初期ポーズの Z 標準偏差 [m]   |
| `std_dev_theta_`  | float  | 5.0 \* M_PI / 180.0                             | 初期ポーズの Theta 標準偏差 [rad]  |
| `length_`         | float  | 4.0                                             | 初期ポーズの X 標準偏差 [m]   |
| `width_`          | float  | 1.8                                             | 初期ポーズの Y 標準偏差 [m]   |
| `height_`         | float  | 2.0                                             | 初期ポーズの Z 標準偏差 [m]   |
| `position_z_`     | float  | 0.0                                             | 初期ポーズの Z 位置 [m]             |
| `velocity_`       | float  | 0.0                                             | 速度 [m/s]                           |

#### BusPose

走行中の車両の姿勢を表現し、車両座標系における現在の姿勢を提供する。

| 名称                  | タイプ   | デフォルト値                                         | 説明                                                 |
| --------------------- | ------ | ------------------------------------------------------- | ------------------------------------------------------- |
| `topic_property_`    | 文字列 | `/simulation/dummy_perception_publisher/object_info` | ダミーオブジェクト情報をパブリッシュするトピック |
| `std_dev_x_`          | 浮動小数 | 0.03                                                   | 初期姿勢に対する X 軸の標準偏差 [m]                    |
| `std_dev_y_`          | 浮動小数 | 0.03                                                   | 初期姿勢に対する Y 軸の標準偏差 [m]                    |
| `std_dev_z_`          | 浮動小数 | 0.03                                                   | 初期姿勢に対する Z 軸の標準偏差 [m]                    |
| `std_dev_theta_`      | 浮動小数 | 5.0 \* M_PI / 180.0                                   | 初期姿勢に対する θ 軸の標準偏差 [rad]                   |
| `length_`             | 浮動小数 | 10.5                                                   | 初期姿勢に対する X 軸の標準偏差 [m]                    |
| `width_`              | 浮動小数 | 2.5                                                    | 初期姿勢に対する Y 軸の標準偏差 [m]                    |
| `height_`             | 浮動小数 | 3.5                                                    | 初期姿勢に対する Z 軸の標準偏差 [m]                    |
| `position_z_`         | 浮動小数 | 0.0                                                    | 初期姿勢に対する Z 軸の位置 [m]                        |
| `velocity_`           | 浮動小数 | 0.0                                                    | 速度 [m/s]                                             |

#### 歩行者姿勢

**概要**

歩行者検出器は、PC（点群）と画像に基づいて、歩行者を検出し、トラッキングします。このコンポーネントは、障害物検知、経路計画、ダイナミックPlanningなどの他のコンポーネントに歩行者情報を提供します。ただし、停车中や駐車中の場合は除きます。

**入力**

* 車両の点群（organized point cloud）
* 車両の画像（RGB画像）
* 自車位置と姿勢（6DoF）
* 制限速度

**出力**

* 検出された歩行者リスト
* 歩行者のバウンディングボックス（3D bounding box）
* 歩行者の速度（東東、南北）
* 歩行者の加速度（東東、南北）
* 歩行者の[後方距離]'post resampling'

**パラメータ**

* 点群の処理時間
* 画像の処理時間
* 検出距離
* 以下の歩行者逸脱量のしきい値:
    * 速度
    * 加速度

| Name              | Type   | Default Value                                        | Description                                     |
| ----------------- | ------ | ---------------------------------------------------- | ----------------------------------------------- |
| `topic_property_` | string | `/simulation/dummy_perception_publisher/object_info` | ダミーオブジェクト情報をパブリッシュするトピック |
| `std_dev_x_`      | float  | 0.03                                                 | 初期位置のX軸標準偏差 [m]       |
| `std_dev_y_`      | float  | 0.03                                                 | 初期位置のY軸標準偏差 [m]       |
| `std_dev_z_`      | float  | 0.03                                                 | 初期位置のZ軸標準偏差 [m]       |
| `std_dev_theta_`  | float  | 5.0 \* M_PI / 180.0                                  | 初期位置のθ軸標準偏差 [rad] |
| `position_z_`     | float  | 0.0                                                  | 初期位置のZ軸位置 [m]                 |
| `velocity_`       | float  | 0.0                                                  | 速度 [m/s]                                  |

#### UnknownPose

このPlanningコンポーネントは、自車位置が不明な状況でAutoware内のナビゲーションスタックを使用するために設計されています。

- 入力
  - 地図上の経路
  - 現在の速度、加速度
  - センサーデータ
- 出力
  - 加速度、操舵角の目標値
  - 減速目標値

このコンポーネントは、以下を使用して経路と自車位置とのずれを推定します。

- `post resampling`に対する経路の逸脱量
- 現在の速度と加速度に対する経路の逸脱量
- センサーデータに対する経路の逸脱量

センサーデータが十分に得られない場合は、コンポーネントは逸脱量を推定できない場合があります。このような場合は、コンポーネントは安全モードに入り、十分なセンサーデータが得られるまで待機します。

このコンポーネントは、以下の交差点の回避に使用できます。

- 信号のない交差点
- 一時停止標識のある交差点

| 名前              | タイプ   | デフォルト値                                        | 説明                                           |
| ----------------- | ------ | ---------------------------------------------------- | ---------------------------------------------- |
| `topic_property_` | 文字列 | `/simulation/dummy_perception_publisher/object_info` | ダミーの物体情報を発行するトピック           |
| `std_dev_x_`      | 浮動小数 | 0.03                                                 | 初期姿勢の X 標準偏差 [m]                     |
| `std_dev_y_`      | 浮動小数 | 0.03                                                 | 初期姿勢の Y 標準偏差 [m]                     |
| `std_dev_z_`      | 浮動小数 | 0.03                                                 | 初期姿勢の Z 標準偏差 [m]                     |
| `std_dev_theta_`  | 浮動小数 | 5.0 \* M_PI / 180.0                                  | 初期姿勢のシータ標準偏差 [rad]               |
| `position_z_`     | 浮動小数 | 0.0                                                  | 初期姿勢の Z 位置 [m]                         |
| `velocity_`       | 浮動小数 | 0.0                                                  | 速度 [m/s]                                     |

#### DeleteAllObjects

| 名前              | タイプ   | デフォルト値                                        | 説明                                     |
| ----------------- | ------ | ---------------------------------------------------- | ----------------------------------------------- |
| `topic_property_` | 文字列 | `/simulation/dummy_perception_publisher/object_info` | ダミーのオブジェクト情報を公開するトピック |

## 制限と前提

Planningシミュレータを使用しています。

## 使用方法

1. rvizを起動し、ツールタブの+を選択します。
   ![select_add](./images/select_add.png)
2. 次のいずれかを選択します: tier4_dummy_object_rviz_plugin を選択してOKを押します。
   ![select_plugin](./images/select_plugin.png)
3. ツールタブの新しい項目を選択します（例では2D Dummy Car）rvizでこれをクリックします。
   ![select_dummy_car](./images/select_dummy_car.png)

### 対話型操作

オブジェクトを対話的に操作できます。

1. rvizで「ツールプロパティ」を選択します。
2. ツールプロパティで対応するオブジェクトタブを選択します。
3. 「対話型」チェックボックスをオンにします。
   ![tool_properties](./images/tool_properties.png)
4. ツールタブでまだ選択していない項目を選択します。
5. キーコマンドは次のとおりです。

| 操作 | キーコマンド |
|---|---|
| 追加 | Shift + 右クリック |
| 移動 | 右クリック長押し + ドラッグアンドドロップ |
| 削除 | Alt + 右クリック |

## Material Designアイコン

このプロジェクトではGoogleの[Material Designアイコン](https://developers.google.com/fonts/docs/material_symbols)を使用しています。これらのアイコンはApache License、Version 2.0の条件に基づいて使用されています。

Material Designアイコンは、アプリケーション、ウェブサイト、およびその他のデジタル製品のユーザーインターフェースを向上させるためにGoogleによって提供されるシンボルのコレクションです。

### ライセンス

Material DesignアイコンはApache License、バージョン2.0のライセンスに基づいています。ライセンスのコピーは次のURLから取得できます。

<http://www.apache.org/licenses/LICENSE-2.0>

適用法によって要求される場合、または書面で合意される場合を除き、ライセンスに基づいて配布されるソフトウェアは、「そのまま」の状態で保証や条件なしに配布されます。ライセンスの明示的または暗示的な許可および制限に関する具体的な言語についてはライセンスを参照してください。

### Acknowledgment

これらのアイコンをコミュニティに提供し、開発者とデザイナーがプロジェクトのビジュアルアピールとユーザーエクスペリエンスを向上させるのに役立てさせてくれたGoogleに感謝の意を表します。


# bytetrack

## 目的

「ByteTrack」というコアアルゴリズムの主たる目的は、複数物体追跡の実行です。
このアルゴリズムは、低検出スコアの検出ボックスもほぼすべて関連付けるため、それを使用することで、偽陰性の数は減少すると予測されます。

[デモビデオ](https://github.com/YoshiRi/autoware.universe/assets/3022416/40f4c158-657e-48e1-81c2-8ac39152892d)

## 内部動作/アルゴリズム

### 引用元

<!-- cspell: ignore Yifu Peize Jiang Dongdong Fucheng Weng Zehuan Xinggang -->

- Yifu Zhang、Peize Sun、Yi Jiang、Dongdong Yu、Fucheng Weng、Zehuan Yuan、Ping Luo、Wenyu Liu、および Xinggang Wang、
  "ByteTrack: Multi-Object Tracking by Associating Every Detection Box", ECCV 2022 の議事録に掲載、[[参照](https://arxiv.org/abs/2110.06864)]
- このパッケージは、[このリポジトリ](https://github.com/ifzhang/ByteTrack/tree/main/deploy/TensorRT/cpp)から Autoware への移植版です
  (ByteTrack の作者による C++ 実装)

### オリジナルコードからの 2D 追跡の変更

論文では、2D 追跡アルゴリズムは単純なカルマンフィルターであるとされているだけです。
オリジナルコードでは、状態ベクトルとして「左上の角」と「アスペクト比」と「サイズ」を使用しています。

アスペクト比は閉塞によって変化する場合があるため、これは不安定になることがあります。
そのため、状態ベクトルとして「左上」と「サイズ」を使用します。

カルマンフィルターの設定は、`config/bytetrack_node.param.yaml` のパラメータで制御できます。

## 入出力

### bytetrack_node

#### 入力

| 名称 | タイプ | 説明 |
|---|---|---|
| `in/rect` | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2Dバウンディングボックスを伴う検出オブジェクト |

#### 自動運転ソフトウェアの日本語ドキュメント

**はじめに**

このドキュメントは、Autoware自動運転ソフトウェアのアーキテクチャ、コンポーネント、およびインターフェースに関する包括的な技術的概要を提供します。

**アーキテクチャ**

Autowareは、モジュール化されたアーキテクチャに基づいており、次の主要なコンポーネントで構成されています。

* **Perception:** センサーデータからオブジェクト、道路、およびその他の環境情報を検出し、追跡します。
* **Planning:** 自車の経路を生成し、周囲の状況に応じて最適化します。
* **Control:** 生成された経路に従って自車を制御します。
* **Localization:** 自車位置と姿勢を推定します。
* **Behavior Planning:** 交通規則や社会的規範に基づいて、自車の挙動を制御します。
* **Map:** 道路網、交通規制、およびその他の環境情報を提供します。

**コンポーネント**

**Perceptionコンポーネント**

* カメラ感知
* LiDAR感知
* レーダー感知

**Planningコンポーネント**

* Motion Planning
* Trajectory Planning
* Path Tracking

**Controlコンポーネント**

* ステアリング制御
* アクセル/ブレーキ制御
* 協調運動制御

**Localizationコンポーネント**

* GPS
* IMU
* オドメトリ

**Behavior Planningコンポーネント**

* 交通信号制御
* 歩行者横断制御
* 車線変更制御

**インターフェース**

Autowareコンポーネントは、ROSインターフェースを使用して相互に通信します。各コンポーネントは、特定のトピックをサブスクライブしてデータを交換します。

**開発ツール**

Autowareは、開発プロセスをサポートするための独自のツールを提供しています。

* **Autoware.AI:** Autowareソフトウェアを構築、テスト、展開するための包括的なプラットフォーム。
* **OpenPlanner:** プランニングアルゴリズムの開発とシミュレーションのためのツール。
* **CARLA:** 自動運転ソフトウェアのテストのためのシミュレーター。

**追加情報**

Autowareコミュニティフォーラムおよびドキュメントページから、追加のサポートとリソースにアクセスできます。

**免責事項**

このドキュメントに記載されている情報は、Autowareプロジェクトの現在の開発状況に基づいています。将来のリリースで変更される場合があります。

| 名称 | タイプ | 説明 |
|---|---|---|
| `out/objects` | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2Dバウンディングボックスを含む検出した物体 |
| `out/objects/debug/uuid` | `tier4_perception_msgs/DynamicObjectArray` | 各物体のユニバーサル一意識別子(UUID) |

### bytetrack_visualizer

#### 入力

| 名称 | 型 | 説明 |
|---|---|---|
| `in/image` | `sensor_msgs/Image` または `sensor_msgs/CompressedImage` | オブジェクト検出を実行する入力画像 |
| `in/rect` | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2D バウンディングボックスを備えた検出オブジェクト |
| `in/uuid` | `tier4_perception_msgs/DynamicObjectArray` | 各オブジェクトの一意の識別子 (UUID) |
| `in/planning_status` | PlanningStatus | Planningモジュールによって計算された、自車位置と周囲の車両の状況 |
| `out/path` | `tier4_planning_msgs/Path` | Planningモジュールによって生成された予定経路 |
| `out/predicted_objects` | `tier4_planning_msgs/PredictedObjects` | 他の車両の予測軌跡 |
| `out/optimized_path` | `tier4_planning_msgs/OptimizedPath` | `post resampling` と走行性改善を適用した最適化された予定経路 |
| `out/published_objects` | `tier4_perception_msgs/PublishedObject` | Autowareが一般的なトピックに公開するオブジェクト |
| `in/odom` | `nav_msgs/Odometry` | 自車位置と速度 |
| `in/detected_lane` | `DetectedLaneArray` | 認識された車線 |
| `out/optimized_pose` | `geometry_msgs/PoseStamped` | 列挙された PlanningStatus の中で「最適化された」もの |
| `out/regular_pose` | `geometry_msgs/PoseStamped` | 列挙された PlanningStatus の中で「一般」のもの |
| `out/emergency_pose` | `geometry_msgs/PoseStamped` | 列挙された PlanningStatus の中で「緊急」のもの |
| `out/velocity_diff` | `geometry_msgs/Twist` | `in/odom` と `optimized_pose` の速度の差 |
| `out/acceleration_diff` | `geometry_msgs/Twist` | `in/odom` と `optimized_pose` の加速度の差 |
| `out/velocity_diff_with_filtered_brake` | `geometry_msgs/Twist` | フィルタリングされたブレーキをかけた `velocity_diff` |
| `out/acceleration_diff_with_filtered_brake` | `geometry_msgs/Twist` | フィルタリングされたブレーキをかけた `acceleration_diff` |
| `out/velocity_diff_with_filtered_gear` | `geometry_msgs/Twist` | フィルタリングされたギアをかけた `velocity_diff` |
| `out/acceleration_diff_with_filtered_gear` | `geometry_msgs/Twist` | フィルタリングされたギアをかけた `acceleration_diff` |

#### 出力

## 自動運転ソフトウェアのドキュメント

### プランニングコンポーネントにおける計画の追従

### 目的

このドキュメントでは、Autowareのプランニングコンポーネントにおける計画の追従に関する技術的詳細について説明します。

### 概要

プランニングコンポーネントは、自車位置と目標位置の差に基づいて、車両の経路を生成します。この経路は、軌道追従コンポーネントによって追従されます。

### 追従アルゴリズム

プランニングコンポーネントにおける追従アルゴリズムは、次のステップで構成されています。

1. **経路の再サンプリング**：計画された経路を一定の間隔で再サンプリングして、車両の現在の位置と方向に合わせます。
2. **前方検索**：再サンプリングされた経路を前方から後方に検索し、現在の位置から最も近い点を見つけます。
3. **横方向の逸脱量の計算**：現在の位置と再サンプリングされた経路上の最も近い点との横方向の距離を計算します。
4. **縦方向の逸脱量の計算**：現在の位置と再サンプリングされた経路上の最も近い点との縦方向の距離を計算します。
5. **追従制御量**：横方向および縦方向の逸脱量に基づいて、軌道追従コンポーネントに追従制御量を生成します。

### 追従制御量

追従制御量は、次の形式です。

* 横方向速度逸脱量（lateral velocity error）
* 横方向加速度逸脱量（lateral acceleration error）
* 縦方向速度逸脱量（longitudinal velocity error）
* 縦方向加速度逸脱量（longitudinal acceleration error）

### パラメータ

追従アルゴリズムのパラメータは、次のとおりです。

* 再サンプリング間隔（resampling interval）
* 前方検索距離（lookahead distance）

### 注意事項

次の点に注意してください。

* 追従アルゴリズムは、 planificatorによって生成された経路に依存しています。
* 追従アルゴリズムは、車両の動的特性とは無関係です。
* 追従アルゴリズムは、リアルタイムで動作するように設計されています。

| 名前        | タイプ                | 説明                                                             |
| ----------- | ------------------- | ------------------------------------------------------------------- |
| `out/image` | `sensor_msgs/Image` | 検出された境界ボックスとそのUUIDが描画された画像                    |

## パラメータ

### bytetrack_node

#### Planningモジュール用のパラメータ

| 名前                  | タイプ | デフォルト値  | 説明                                    |
| --------------------- | ---- | ------------- | ------------------------------------------- |
| `track_buffer_length` | int  | 30             | トラッキングが失われたと見なされるフレーム数 |

### bytetrack_visualizer

※: このドキュメントはAutoware.Autoの開発ドキュメントの一部です。


| 名称        | 種類 | デフォルト値 | 説明                                                                                     |
| --------- | ---- | ------------- | ---------------------------------------------------------------------------------------------- |
| `use_raw` | bool | false         | ノードが `sensor_msgs/Image` または `sensor_msgs/CompressedImage` を入力として切り替える場合のフラグ |

## 想定/既知の制限

## 参考リポジトリ

- <https://github.com/ifzhang/ByteTrack>

## ライセンス

`lib` ディレクトリ下のコードは [元のコード](https://github.com/ifzhang/ByteTrack/tree/72ca8b45d36caf5a39e949c6aa815d9abffd1ab5/deploy/TensorRT/cpp) からコピーして修正しています。
元のコードは、以下のとおり記載されている MIT ライセンスに則り、この移植パッケージは Apache License 2.0 にて提供されます。

> MIT License
>
> Copyright (c) 2021 Yifu Zhang
>
> これにより、本ソフトウェアおよび関連するドキュメントファイル（「ソフトウェア」）の複製物を取得するすべての人に無償で許可が与えられます。制限なくソフトウェアを使用、複製、変更、マージ、公開、配布、サブライセンス付与、および/または販売し、また、ソフトウェアが提供される人にサブライセンス付与を行う権利を含むがこれに限定されません。ただし、次の条件に従うものとします。
>
> 上記の著作権表示およびこの通知は、ソフトウェアのすべての複製または重要な部分に含まれる必要があります。
>
> ソフトウェアは明示的または黙示的に保証されることなく、「現状のまま」提供されます。商品性、特定の目的への適合性、非侵害の保証を含むがこれらに限定されない保証は含まれます。いかなる場合も、著作権者または権利者は、契約、不法行為、またはその他の行為に関連して、またはその結果、ソフトウェアまたはソフトウェアの使用または他の取り扱いから生じる請求、損害、またはその他の責任について責任を負いません。


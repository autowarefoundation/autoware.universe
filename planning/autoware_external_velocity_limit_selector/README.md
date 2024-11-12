## 外部速度制限セレクタ

## 目的

`external_velocity_limit_selector_node` は、外部速度制限の一貫性を保つノードです。このモジュールは、次のメッセージをサブスクライブします。

1. **API** が送信した速度制限コマンド
2. **Autoware 内部モジュール** が送信した速度制限コマンド

VelocityLimit.msg には、**最大速度**だけでなく、減速時の **加速度/ジャーク制約** に関する情報も含まれます。`external_velocity_limit_selector_node` は、API と Autoware 内部モジュールから送信されたすべての減速ポイントと最大速度を保護する **最も厳しい速度制限** を計算するために、最も低い速度制限と最も高いジャーク制約を統合します。

![セレクタアルゴリズム](./image/external_velocity_limit_selector.png)

## 内部動作 / アルゴリズム

WIP

<!-- このパッケージの動作を説明します。フローチャートや図は適しています。必要なだけサブセクションを追加してください。

例:
  ### フローチャート

  ...(PlantUML など)

  ### ステート遷移

  ...(PlantUML など)

  ### ターゲット障害物のフィルタリング方法

  ...

  ### 軌道の最適化方法

  ...
-->

## 入力
```
#!cpp
  // API から受信した速度制限
  VelocityLimit api_limit;

  // planner から受信した速度制限
  VelocityLimit planner_limit;

  // current pose
  geometry_msgs::PoseStamped current_pose;
```

| 名称                                                   | タイプ                                            | 説明                                         |
| ------------------------------------------------------ | ------------------------------------------------ | -------------------------------------------- |
| `~input/velocity_limit_from_api`                     | tier4_planning_msgs::VelocityLimit              | APIからの速度制限                           |
| `~input/velocity_limit_from_internal`                | tier4_planning_msgs::VelocityLimit              | Autoware内部モジュールからの速度制限       |
| `~input/velocity_limit_clear_command_from_internal` | tier4_planning_msgs::VelocityLimitClearCommand | Autoware内部モジュールからの速度制限クリアコマンド |

## 出力

**Global Path Plan**
* グローバルパス計画

**Local Trajectory Plan**
* ローカルトラジェクトリピュラン

**Planning**
* Planning

**Map Matching**
* マップマッチング

**Autoware API**
* Autoware API

**Behavior Planning**
* 行動計画

**Motion Planning**
* モーション計画

**Control**
* 制御

**Perception**
* 知覚

**Vehicle Dynamics**
* 車両運動力学

**Sensor**
* センサー

**Localization**
* ローカライズ

**Post Processing**
* ポスト処理

**Planning**
* Planning

**Plotting**
* プロッティング

**Best Trajectory Search**
* 最適トラジェクトリサーチ

**Clustering**
* クラスタリング

**Clustering Parameters**
* クラスタリングパラメータ

**Post-resampling**
* `post-resampling`

**Downsampling**
* ダウンサンプリング

**Interpolation**
* 補間

**Velocity Violation Threshold**
* 速度逸脱量閾値

**Acceleration Violation Threshold**
* 加速度逸脱量閾値

**Jerk Violation Threshold**
* ジャーク逸脱量閾値

**Snap Violation Threshold**
* スナップ逸脱量閾値

**Path Smoothing**
* パススムージング

**Uniform B-Spline Smoothing**
* Uniform B-Splineスムーージング

**LSF Smoothing**
* LSFスムージング

**Self-Motion Smoothing**
* 自車運動スムージング

**Generate Trajectory**
* トラジェクトリ生成

**Generate Path**
* パス生成

**Generate Sim Trajectories**
* シミュレーション用トラジェクトリ生成

**Publish Drive Trajectory**
* 駆動用トラジェクトリ発行

**Publish Planning Visualization**
* Planningビジュアライゼーション発行

**Publish Trajectory**
* トラジェクトリ発行

**Publishing**
* 発行

**Publish Pose**
* 自車位置発行

**Publish Stop Trajectory**
* 停止用トラジェクトリ発行

**Publish Turn Signal**
* 方向指示器発行

**Publish Vehicle State**
* 車両状態発行

**Subscribe Map**
* マップ購読

**Subscribe Pose**
* 自車位置購読

**Subscribe Stop Sign**
* 停止標識購読

**Subscription**
* 購読

**Target Planning**
* ターゲットPlanning

**Trajectory Converter**
* トラジェクトリコンバータ

**Trajectory Optimization**
* トラジェクトリ最適化

**Twist2D**
* Twist2D

**Vehicle Model**
* 車両モデル

| 名前                   | 型                               | 説明                                         |
| ---------------------- | ---------------------------------- | ------------------------------------------------ |
| `~output/max_velocity` | tier4_planning_msgs::VelocityLimit | 自車位置における最も厳格な速度制限の最新情報 |

## パラメータ

| パラメーター         | タイプ   | 説明                                 |
| ----------------- | ------ | --------------------------------------- |
| `max_velocity`    | double | デフォルト最大速度 [m/s]                 |
| `normal.min_acc`  | double | 最小加速度 [m/ss]                       |
| `normal.max_acc`  | double | 最大加速度 [m/ss]                       |
| `normal.min_jerk` | double | 最小ジャーク [m/sss]                     |
| `normal.max_jerk` | double | 最大ジャーク [m/sss]                     |
| `limit.min_acc`   | double | 守られるべき最小加速度 [m/ss]           |
| `limit.max_acc`   | double | 守られるべき最大加速度 [m/ss]           |
| `limit.min_jerk`  | double | 守られるべき最小ジャーク [m/sss]         |
| `limit.max_jerk`  | double | 守られるべき最大ジャーク [m/sss]         |

## 仮定 / 既知の制限事項

<!-- 実装の仮定と制限事項を記載します。

例:
  このアルゴリズムは障害物が動かないことを前提としています。従って、車両が障害物の回避を開始してから障害物が急速に移動した場合、障害物と衝突する可能性があります。
  また、このアルゴリズムは死角を考慮しません。一般に、あまりにも近い障害物はセンシング性能の制限により検出できません。そのため、障害物に対して十分な余裕を持たせてください。
-->

## (オプション) エラー検出と処理

<!-- エラーを検出する方法と、それらから回復する方法を記載します。

例:
  このパッケージは最大20個の障害物を処理できます。それ以上の障害物が検出された場合、このノードは処理を放棄して診断エラーを出力します。
-->

## (オプション) パフォーマンス特性

<!-- 複雑性などのパフォーマンス情報を記載します。ボトルネックにならない場合は、必須ではありません。

例:
  ### 複雑性

  このアルゴリズムはO(N)です。

  ### 処理時間

  ...
-->

## (オプション) 参考文献 / 外部リンク

<!-- 実装時に参照したリンクを記載します。

例:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (オプション) 将来の拡張 / 未実装部分

<!-- このパッケージの将来の拡張を記載します。

例:
  現在、このパッケージはチャタリング障害物を適切に処理できません。これを改善するために、認識レイヤーにいくつかの確率的フィルタを追加することを計画しています。
  また、グローバルであるべきいくつかのパラメータがあります（例：車両サイズ、最大操舵角など）。これらはリファクタリングされ、グローバルパラメータとして定義されるため、さまざまなノード間で同じパラメータを共有できます。
-->


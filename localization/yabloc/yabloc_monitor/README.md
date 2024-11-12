# yabloc_monitor

YabLoc モニターは YabLoc Localization システムの状態を監視するノードです。YabLoc Localization システムの状態を監視し、その状態を診断として発行するラッパーノードです。

## 機能

### 可用性

このノードは、YabLoc の最終出力姿勢を監視して YabLoc の可用性を検証します。

### その他

追加予定です。

## インターフェイス

### 入力

| 名前                  | 型                        | 説明                     |
| --------------------- | --------------------------- | ------------------------------- |
| `~/input/yabloc_pose` | `geometry_msgs/PoseStamped` | YabLocの最終出力姿勢 |

### 出力

この文書は、AutowareのPath PlanningモジュールのためのPythonリファレンスガイドです。

**目的**

本ドキュメントの目的は、Path Planningモジュールの各クラス、関数、定数に関して、その意味と使用方法を明確にすることです。

**前提条件**

読者は、Pythonプログラミング言語、モジュール性、およびオブジェクト指向プログラミングの概念に精通している必要があります。また、Autowareフレームワークの基本的な理解も有益です。

**構成**

このドキュメントは、次のセクションで構成されています。

- **モジュール**
- **クラス**
- **関数**
- **定数**
- **付録**

**モジュール**

Path Planningモジュールは、`autoware.planning`モジュール内に格納されています。このモジュールには、パス計画アルゴリズム、軌跡生成機能、および障害物回避ルーチンを実装するクラスと関数が含まれています。

**クラス**

Path Planningモジュールには、次の主要なクラスが含まれています。

- **Planner:** Planningアルゴリズムのベースクラス
- **TrajectoryGenerator:** 軌跡を生成するクラス
- **ObstacleAvoidance:** 障害物を回避するためのルーチンをカプセル化するクラス

**関数**

Path Planningモジュールには、次の主要な関数が含まれています。

- `plan_path(current_pose, goal_pose, obstacles)`: 与えられた自車位置、ゴール位置、および障害物を基にパスを計画する
- `generate_trajectory(path, velocity, acceleration)`: 与えられたパス、速度、および加速度に基づいて軌跡を生成する
- `avoid_obstacles(trajectory, obstacles)`: 与えられた軌跡と障害物を基に障害物を回避する

**定数**

Path Planningモジュールには、次の主要な定数が含まれています。

- `MAX_VELOCITY`: 最適速度
- `MAX_ACCELERATION`: 最大加速度
- `MIN_DISTANCE_TO_OBSTACLE`: 障害物に対する最小安全距離

**付録**

付録には、Path Planningモジュールの使用方法に関する追加情報が含まれています。

* `post resampling`処理に関するセクションを含んでいます。

| Name           | Type                              | Description         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 診断結果出力       |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_monitor/schema/yabloc_monitor.schema.json", "ja") }}


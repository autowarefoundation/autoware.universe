## yabloc_common

このパッケージには地図に関連するいくつかの実行可能ノードが含まれます。また、yabloc用の一般的なライブラリを提供します。

- [ground_server](#ground_server)
- [ll2_decomposer](#ll2_decomposer)

## ground_server

### 目的

レーンレット2から路面の高さや傾きを推定します。

### 入出力

#### 入力

| 名称               | 種別                                   | 説明                                 |
| ------------------ | -------------------------------------- | ----------------------------------- |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | ベクタマップ                           |
| `input/pose`       | `geometry_msgs::msg::PoseStamped`       | 自車位置                              |

#### 出力

```
## 自動運転ソフトウェアの概要

### 要件

このソフトウェアは、以下のような要件を満たすように設計されています。

- 安全かつ信頼性の高い自動運転
- さまざまな道路状況への適応性
- 低消費電力の計算処理
- GUIベースのユーザフレンドリーなインターフェイス

### アーキテクチャ

このソフトウェアは、以下のようなモジュール構造のアーキテクチャを採用しています。

- **Planning:** 経路計画、障害物回避、速度制御など、自動運転の主要なタスクを実行します。
- **Localization:** 自車位置と姿勢をリアルタイムで特定します。
- **Perception:** センサーデータから周囲環境を認識します。
- **Control:** ブレーキ、アクセル、ステアリングを制御して、計画された経路に従います。
- **GUI:** ユーザーにソフトウェアの設定やシステムステータスに関する情報を提供します。

### 主な機能

このソフトウェアは以下の主要機能を提供します。

- **Adaptive Cruise Control (ACC):** 車間距離を維持して速度を自動調整します。
- **Lane Keeping Assist (LKA):** 車線を維持して逸脱を防ぎます。
- **Lane Departure Warning (LDW):** 車線からの逸脱を検出し警告します。
- **Collision Warning System (CWS):** 前方障害物を検出し衝突の可能性について警告します。
- **Autonomous Emergency Braking (AEB):** 衝突を回避するために自動的にブレーキをかけます。

### 使用方法

このソフトウェアを使用するには、次の手順に従います。

1. ソフトウェアをダウンロードしてインストールします。
2. センサーデータをソフトウェアに接続します。
3. GUIを使用して適切な設定を行います。
4. 自動運転モードを有効にします。

### トラブルシューティング

問題が発生した場合は、以下の一般的なトラブルシューティング手順に従います。

1. センサーデータの接続を確認します。
2. GUIでセンサーデータが正しく表示されていることを確認します。
3. Planningモジュールの設定を確認します。
4. Controlモジュールの設定を確認します。
5. `post resampling`手法を使用している場合は、適切なパラメータが設定されていることを確認します。

### サポート

サポートが必要な場合は、Autowareコミュニティフォーラムにアクセスしてください。

### 注意事項

このソフトウェアは開発中のソフトウェアであり、すべての状況で確実に機能するわけではありません。自動運転中は常に周囲に注意を払い、必要に応じて手動操作を行ってください。
```

| 名称                   | タイプ                                 | 説明                                                                     |
| ----------------------- | ------------------------------------- | ------------------------------------------------------------------------------- |
| `output/ground`         | `std_msgs::msg::Float32MultiArray` | 推定された路面パラメーター。x、y、z、normal_x、normal_y、normal_z を含む。 |
| `output/ground_markers` | `visualization_msgs::msg::Marker`  | 推定された路面の状態の可視化                                                 |
| `output/ground_status`  | `std_msgs::msg::String`            | 路面状態の推定状態のログ                                                  |
| `output/height`         | `std_msgs::msg::Float32`           | 高度                                                                        |
| `output/near_cloud`     | `sensor_msgs::msg::PointCloud2`    | Lanelet2 から抽出され、路面の傾斜推定に使用される点群                         |

### パラメーター

{{ json_to_markdown("localization/yabloc/yabloc_common/schema/ground_server.schema.json") }}

## ll2_decomposer

### 目的

このノードは、レーンのマークとyablokに関する要素をlanelet2から抽出します。

### 入出力

#### 入力

| 名称 | タイプ | 説明 |
|---|---|---|
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | vectorマップ |

#### 出力

Autowareドキュメント（日本語訳）

**はじめに**

本ドキュメントでは、さまざまな自動運転シナリオにおけるソフトウェアアーキテクチャとコンポーネントについて説明します。

**ソフトウェアアーキテクチャ**

Autowareのソフトウェアアーキテクチャは、次の主要なコンポーネントで構成されています。

- **Perception（認識）:** 車両の周囲環境を感知します。
- **Prediction（予測）:** 周囲の物体の動きを予測します。
- **Planning（計画）:** 車両の経路と操縦を計画します。
- **Control（制御）:** 車両の運動を制御します。

**Planningコンポーネント**

Planningコンポーネントは、次のサブコンポーネントで構成されています。

- **Path Planning（パス計画）:** 自車位置に基づいて安全な経路を生成します。
- **Behavior Planning（挙動計画）:** 車両の加速、減速、操舵などの挙動を生成します。

**Planningコンポーネントの機能**

Planningコンポーネントは、次の機能を実行します。

* Perceptionによって提供される環境情報を使用して、安全な経路を計画します。
* Predictionによって提供される物体の動き予測を使用して、計画された経路に沿った衝突の可能性を評価します。
* 車両の速度、加速度逸脱量、操舵逸脱量などの制約を考慮して、実行可能な挙動を生成します。

**Planningコンポーネントの入力**

Planningコンポーネントの入力には、次のものが含まれます。

* 自車位置
* Perceptionによって提供される環境データ
* Predictionによって提供される物体の動き予測
* 車両の制限事項

**Planningコンポーネントの出力**

Planningコンポーネントの出力には、次のものが含まれます。

* 安全な経路
* 車両の加速、減速、操舵の挙動

**追加のリソース**

* [Autoware 公式ドキュメント](https://docs.autoware.org/)

| 名称                       | タイプ                               | 説明                                 |
| -------------------------- | ------------------------------------- | -------------------------------------- |
| `output/ll2_bounding_box`  | `sensor_msgs::msg::PointCloud2`        | レーンレット2 から抽出したバウンディングボックス |
| `output/ll2_road_marking`  | `sensor_msgs::msg::PointCloud2`        | レーンレット2 から抽出した路面標示 |
| `output/ll2_sign_board`    | `sensor_msgs::msg::PointCloud2`        | レーンレット2 から抽出した標識 |
| `output/sign_board_marker` | `visualization_msgs::msg::MarkerArray` | 可視化された標識                |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_common/schema/ll2_decomposer.schema.json") }}


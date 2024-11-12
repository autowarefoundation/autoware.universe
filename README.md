# Autoware Universe

## Autoware Universeへようこそ

Autoware UniverseはAutowareエコシステムの基礎的な柱として、自動運転技術における中核的な機能を強化する重要な役割を果たします。
このリポジトリはAutoware Core/Universeコンセプトの中核的な要素であり、自動運転車の機能を大幅に拡張する広範なパッケージを管理しています。

![autoware_universe_front](docs/assets/images/autoware_universe_front.png)

## 入門

Autowareの広大な世界に飛び込み、Autoware Universeが全体像の中でどのように適合するかを理解するには、[Autowareドキュメント](https://autowarefoundation.github.io/autoware-documentation/)から始めることをお勧めします。このリソースはAutowareエコシステムの包括的な概要を提供し、そのコンポーネント、機能、開発の始め方を説明しています。

### Autoware Universeのドキュメントを探る

Autoware Universeコンポーネントの詳細を探りたい場合は、MKDocsで展開された[Autoware Universeのドキュメント](https://autowarefoundation.github.io/autoware.universe/)で詳細な情報を見つけることができます。

## コードカバレッジメトリクス

以下の表は、Autoware Universe全体およびサブコンポーネントのそれぞれのカバレッジ率を示しています。

### プロジェクト全体のカバレッジ

[![codecov](https://codecov.io/github/autowarefoundation/autoware.universe/graph/badge.svg?token=KQP68YQ65D)](https://codecov.io/github/autowarefoundation/autoware.universe)

### コンポーネントごとのカバレッジ

バッジをクリックしてcodecovのウェブサイトに移動すると、詳細を確認できます。

| コンポーネント | 対応範囲 |
| ------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Common       | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Common%20Packages&query=$.[0].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Common%20Packages)             |
| Control      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Control%20Packages&query=$.[1].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Control%20Packages)           |
| Evaluator    | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Evaluator%20Packages&query=$.[2].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Evaluator%20Packages)       |
| Launch       | TBD                                                                                                                                                                                                                                                                                                             |
| Localization | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Localization%20Packages&query=$.[4].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Localization%20Packages) |
| Map          | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Map%20Packages&query=$.[5].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Map%20Packages)                   |
| Perception   | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Perception%20Packages&query=$.[6].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Perception%20Packages)     |
| Planning     | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Planning%20Packages&query=$.[7].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Planning%20Packages)         |
| Sensing      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Sensing%20Packages&query=$.[8].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Sensing%20Packages)           |
| Simulator    | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Simulator%20Packages&query=$.[9].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Simulator%20Packages)       |
| System       | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=System%20Packages&query=$.[10].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=System%20Packages)            |
| Vehicle      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Vehicle%20Packages&query=$.[11].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Vehicle%20Packages)          |

**自動運転ソフトウェアに関するドキュメント**

## Self-Driving Software Architecture

### Planningモジュール

#### ユースケースと要件

* 自車位置と目的地の認識
* 安全かつ効率的な経路の作成
* 障害物回避と衝突予測
* 高速道路と市街地でのナビゲーション対応

#### 実装の詳細

* 動的パスプランニングアルゴリズム（例：A*、D*ライト）を使用したリアルタイム経路生成
* 車載センサー（LiDAR、カメラ、レーダー）からのデータ統合
* 障害物検出と逸脱量リスク評価のための物体検出アルゴリズム
* 高精度地図データの活用

#### パフォーマンスの測定基準

* 目的地への到着時間
* 走行距離と燃料消費量
* 障害物との衝突回避率
* 加速度逸脱量、速度逸脱量

### Perceptionモジュール

#### ユースケースと要件

* 車両周囲の環境認識
* 動的および静的障害物の検出と追跡
* 物体の形状、サイズ、速度の推定
* 交通状況のモニタリングと予測

#### 実装の詳細

* LiDARとカメラからのデータを融合した物体検出と追跡アルゴリズム
* 領域ベースの物体検出器（例：YOLO、Faster R-CNN）
* ニューラルネットワークを使用した特徴抽出と分類
* 地図データの活用とセンシングデータへの投影

#### パフォーマンスの測定基準

* 物体検出の精度（mAP）
* 物体追跡の精度（IDF1）
* 処理スループット（FPS）
* エラー検出率（例：偽検出率、偽陰性率）

### Controlモジュール

#### ユースケースと要件

* アクセル、ブレーキ、ステアリングの制御
* 経路計画からの命令の追従
* 障害物回避のための緊急回避動作
* 車両の安定性とハンドリングの維持

#### 実装の詳細

* PID制御器を使用したフィードバック制御
* モデル予測制御（MPC）を使用した最適制御
* 先行車両追従のための適応制御アルゴリズム
* 緊急回避のための最適化されたパスクレプランニング

#### パフォーマンスの測定基準

* 経路からの逸脱量
* アクセル、ブレーキ、ステアリング操作の応答性
* エネルギー効率（例：燃料消費量）

### Sensingモジュール

#### ユースケースと要件

* 車両周囲の環境データの収集
* LiDAR、カメラ、レーダーなど、複数のセンサーのデータ統合
* データの収集と'post resampling'のリアルタイム処理
* センサーキャリブレーションとエラー補正

#### 実装の詳細

* ROS（Robot Operating System）ベースのセンサーデータ統合システム
* センサーデータの同期と'post resampling'のリアルタイム処理
* レーザー、RGB、深度データの融合技術
* 固有のセンサーデバイスドライバの開発

#### パフォーマンスの測定基準

* データの収集範囲と精度
* データの'post resampling'と同期化の精度
* センサーキャリブレーションの品質
* 処理スループット（FPS）

### Autowareとの統合

* ROSベースのアーキテクチャによるAutowareへのシームレスな統合
* モジュールの再利用性と拡張性
* オープンソースコミュニティによる継続的な開発とサポート


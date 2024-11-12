## autoware_localization_error_monitor

## 目的

<p align="center">
<img src="./media/diagnostics.png" width="400">
</p>

autoware_localization_error_monitorは、位置推定の結果の不確かさを監視することで位置推定のエラーを診断するためのパッケージです。
このパッケージは次の2つの値を監視します。

- 信頼楕円の長い半径のサイズ
- 横方向（車体フレーム）に沿った信頼楕円のサイズ

## 入出力

### 入力

| 名前         | タイプ                      | 説明         |
| ------------ | ------------------------- | ------------------- |
| `input/odom` | `nav_msgs::msg::Odometry` | 自車位置結果 |

### 出力

**自動運転ソフトウェアの設計**

**概要**

このドキュメントは、自動運転ソフトウェアの設計の概要を説明します。以下に含まれる内容を示します。

- アーキテクチャ
- 主要コンポーネント
- アルゴリズム
- 実装

**アーキテクチャ**

Autowareのアーキテクチャは、モジュラー化されたレイヤー構造に基づいています。各レイヤーは、特定のタスクを担当するモジュールで構成されます。このアーキテクチャにより、柔軟性、拡張性、保守性が向上します。

**主要コンポーネント**

主要コンポーネントには、以下が含まれます。

- Planningコンポーネント
- Perceptionコンポーネント
- Controlコンポーネント
- Vehicle Interfaceコンポーネント

**アルゴリズム**

Autowareは、以下を含むさまざまなアルゴリズムを使用しています。

- パス計画
- トラフィック予測
- オブジェクト検出
- クラシフィケーション

**実装**

Autowareは、ROS (Robot Operating System) を実装しています。ROSは、ロボットソフトウェア開発のためのオープンソースのフレームワークです。このフレームワークにより、モジュールの再利用、拡張、テストが容易になります。

**評価**

Autowareは、シミュレーションと実際の道路テストの両方で評価されています。結果は、Autowareがさまざまなシナリオで安全かつ効率的に動作することを示しています。

**使用例**

Autowareは、以下を含むさまざまな用途で使用できます。

- 自動運転車
- ロボット
- ドローン

**サポート**

Autowareは、アクティブなコミュニティによってサポートされています。コミュニティでは、ドキュメント、フォーラム、チュートリアルを介してサポートを提供しています。

**貢献**

Autowareはオープンソースプロジェクトです。貢献に興味がある場合は、GitHubリポジトリをご覧ください。

**詳細情報**

Autowareに関する詳細情報は、以下のリソースを参照してください。

- 公式ウェブサイト: https://www.autoware.ai
- GitHubリポジトリ: https://github.com/autowarefoundation/autoware.ai

| 名称                   | 型                                    | 説明         |
| ---------------------- | --------------------------------------- | ------------------- |
| `debug/ellipse_marker` | `visualization_msgs::msg::Marker`       | 楕円マーカー      |
| `diagnostics`          | `diagnostic_msgs::msg::DiagnosticArray` | 診断出力        |

## パラメータ

{{ json_to_markdown("localization/autoware_localization_error_monitor/schema/localization_error_monitor.schema.json", True) }}


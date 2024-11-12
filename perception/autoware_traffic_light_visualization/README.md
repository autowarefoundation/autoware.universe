# autoware_traffic_light_visualization

## 目的

`autoware_traffic_light_visualization`は、2つの可視化ノードを含むパッケージです。

- **traffic_light_map_visualizer**は、rviz上で交通信号の色状態と位置をマーカーとして表示するノードです。
- **traffic_light_roi_visualizer**は、下の図に示すように、交通信号認識ノードの結果（交通信号の状態、位置、分類確率）を入力画像に描き、パブリッシュするノードです。

![交通信号ROI可視化](./images/roi-visualization.png)

## 内部動作 / アルゴリズム

## 入出力

### traffic_light_map_visualizer

#### 入力

| 名前                 | タイプ                                                 | 説明              |
| -------------------- | ---------------------------------------------------- | ----------------- |
| `~/input/tl_state`   | `tier4_perception_msgs::msg::TrafficLightGroupArray` | 信号機の状態      |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin`              | ベクターマップ      |

#### 出力

**自動運転ソフトウェア**

**概要**

このドキュメントでは、自律走行車両の開発に採用される自動運転ソフトウェアアーキテクチャについて説明します。このアーキテクチャは、業界標準であるAutowareをベースに設計されており、Perception（知覚）、Planning（プランニング）、Control（制御）の3つの主要コンポーネントで構成されています。

**アーキテクチャ**

**1. Perception**

Perceptionコンポーネントは、センサーデータ（カメラ、レーダー、LiDARなど）を使用して、車両の周囲環境を検出し、解釈します。このデータから、以下の情報が生成されます。

* オブジェクト検出（車両、歩行者、障害物）
* 車両の周辺認識（自車位置、車線、交通標識）
* 環境マッピング

**2. Planning**

Planningコンポーネントは、Perceptionコンポーネントによって生成された情報を基に、車両の経路を計画します。この経路は、以下の制約を考慮して生成されます。

* 交通規則
* 速度制限
* 障害物回避
* ドライバの意図（例：ウインカーからの入力）

**3. Control**

Controlコンポーネントは、Planningコンポーネントによって生成された経路に基づいて、車両を制御します。これには以下の機能が含まれます。

* ステアリング角制御
* スロットル制御
* ブレーキ制御

**リアルタイム処理**

このアーキテクチャは、リアルタイムで処理されます。つまり、ソフトウェアはセンサーデータを受け取り、環境を認識し、経路を計画し、車両を制御するために継続的に実行されます。

**エラー処理**

このアーキテクチャには、以下のエラー処理メカニズムが含まれています。

* センサーデータの冗長性（複数のセンサーを使用）
* Planningの冗長性（複数のPlanningアルゴリズムを使用）
* Controlの冗長性（複数のアクチュエータを使用）

**テストと検証**

このアーキテクチャのテストと検証は、以下の方法で行われます。

* シミュレーション
* 屋外試験（公道上のテスト）
* `post resampling`の解析

**業界における関連性**

このアーキテクチャは、自律走行車両の業界で広く採用されています。そのモジュール性と拡張性が、さまざまな車両タイプや運転シナリオに対応することを可能にします。

**用語集**

* Perception：車両の周囲環境の認識と解釈
* Planning：車両の経路計画
* Control：車両の制御とアクチュエーション
* `post resampling`：シミュレーションデータを使用して、実際に収集されたデータのテストを行う手法
* velocity逸脱量：速度逸脱の量
* acceleration逸脱量：加速度逸脱の量

| 名称                     | タイプ                                   | 説明                                          |
| ------------------------ | -------------------------------------- | ---------------------------------------------------- |
| `~/output/traffic_light` | `visualization_msgs::msg::MarkerArray` | 信号機のステータスを示すマーカー配列 |

### traffic_light_roi_visualizer

#### 入力

| 名目                          | タイプ                                               | 説明                                                  |
| ----------------------------- | -------------------------------------------------- | -------------------------------------------------------- |
| `~/input/tl_state`            | `tier4_perception_msgs::msg::TrafficLightArray`    | 信号状態                                               |
| `~/input/image`               | `sensor_msgs::msg::Image`                          | 知覚カメラでキャプチャした画像                          |
| `~/input/rois`                | `tier4_perception_msgs::msg::TrafficLightRoiArray` | `traffic_light_fine_detector` によって検出された ROI |
| `~/input/rough/rois` (オプション) | `tier4_perception_msgs::msg::TrafficLightRoiArray` | `traffic_light_map_based_detector` によって検出された ROI |

#### 出力

**自動運転ソフトウェアに関するドキュメント**

本ドキュメントは、AutowareのBehavior Planningコンポーネントの設計概要を示しています。

**概要**

Behavior Planningコンポーネントは、他のモジュールから提供された情報に基づいて、自律走行車の安全かつ効率的な経路を計画します。このコンポーネントは、次の主要なタスクを実行します。

- 目標軌道の生成
- 経路追従の計画
- 障害物の回避
- 安全性の評価

**設計**

Behavior Planningコンポーネントは、以下のモジュールで構成されています。

- **Goal Planningモジュール:** 目標軌道の生成を担当します。
- **Path Planningモジュール:** 経路追従の計画を担当します。
- **Obstacle Avoidanceモジュール:** 障害物の回避を担当します。
- **Safety Assessmentモジュール:** 安全性の評価を担当します。

**動作**

Behavior Planningコンポーネントは、以下の順序で動作します。

1. **目標軌道の生成:** Goal Planningモジュールは、自車位置と目標の位置に基づいて目標軌道を生成します。
2. **経路追従の計画:** Path Planningモジュールは、目標軌道に従って、障害物や他の交通参加者を考慮した経路を計画します。
3. **障害物の回避:** Obstacle Avoidanceモジュールは、経路上の障害物を検出し、それらを回避する経路変更を計画します。
4. **安全性の評価:** Safety Assessmentモジュールは、計画された経路の安全性を評価し、速度逸脱量や加速度逸脱量が適切な範囲内であることを確認します。

**インターフェース**

Behavior Planningコンポーネントは、他のAutowareコンポーネントと以下のインターフェースで通信します。

- **Perceptionモジュール:** 障害物や他の交通参加者に関する情報を提供します。
- **Localizationモジュール:** 自車位置に関する情報を提供します。
- **Controlモジュール:** 生成された経路を実行します。

**検証**

Behavior Planningコンポーネントは、シミュレーションと実車テストの両方で検証されています。このコンポーネントは、さまざまな運転状況において、安全かつ効率的に動作することが実証されています。

**今後の改善**

Behavior Planningコンポーネントは、継続的に改善されています。今後の改善には、次のようなものが含まれます。

- 計画の品質の向上
- 計算効率の向上
- 安全性に関する評価能力の向上

| Name              | Type                          | Description             |
| ------------------ | ---------------------------- | ----------------------- |
| `~/output/image`   | `sensor_msgs::msg::Image`    | ROI付き出力画像        |

## パラメータ

### traffic_light_map_visualizer

なし

### traffic_light_roi_visualizer

#### ノードパラメータ

{{json_to_markdown("perception/autoware_traffic_light_visualization/schema/traffic_light_visualization.schema.json")}}

## 仮定/既知の制限

## (任意) エラー検知と処理

## (任意) パフォーマンス特性

## (任意) 参照/外部リンク

## (任意) 将来の拡張/未実装部分


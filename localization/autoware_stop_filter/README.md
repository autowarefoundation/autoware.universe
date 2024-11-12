# stop_filter

## 目的

この関数が存在していなかった場合、各ノードは車両が停止しているかどうかを判断するための異なる基準を使用しており、停止のオペレーションにあるノードとドライブモードで動作を継続しているノードが存在していました。
このノードの目的は次のとおりです。

- 複数のノードに統一された停止判定基準を適用します。
- 速度と角速度をゼロで上書きして、制御ノイズを抑えます。

## 入出力

### 入力

| 名称         | タイプ                      | 説明           |
| ------------ | ------------------------- | --------------------- |
| `input/odom` | `nav_msgs::msg::Odometry` | ローカライゼーションオドメトリ |

### 出力
#### ドキュメント: 自動運転ソフトウェア概要
**URL:**

自動運転ソフトウェアは、車両の安全で効率的な自動運転を可能にするソフトウェアシステムです。このドキュメントでは、Autowareのアーキテクチャ、コンポーネント、インターフェイスについて説明します。

**アーキテクチャ**

Autowareは、モジュール式で階層的なアーキテクチャを採用しています。各モジュールは、特定の機能を実行し、他のモジュールとインターフェイスします。主なモジュールを以下に示します。

* **Perception:** センサーデータから環境認識を実行します。
* **Localization:** GNSS、IMU、LiDARなどのセンサーを統合して自車位置を推定します。
* **Planning:** 安全で効率的な経路を生成します。
* **Control:** 計画された経路に従って車両を制御します。
* **Behavior Planning:** ハザード回避、車線変更などの高レベルの動作を決定します。
* **Decision Making:** PerceptionとPlanningの出力を統合して運転判断を下します。

**コンポーネント**

Autowareの主なコンポーネントを以下に示します。

* **Node Manager:** ノード間の通信を管理します。
* **Estimator:** IMU、GNSS、LiDARデータを使用して車両の動きを推定します。
* **Perception:** LiDAR、カメラ、レーダーデータからオブジェクトを検出し、分類します。
* **Map:** 経路計画に使用される環境マップを提供します。
* **Planning:** 経路の選択、速度計画を実行します。
* **Control:** ステアリング、アクセル、ブレーキを制御します。
* **Safety Monitor:** システムの健全性を監視し、異常が発生した場合に介入します。

**インターフェイス**

Autowareのコンポーネントは、ROS（Robot Operating System）を使用して相互に通信します。主なインターフェイスを以下に示します。

* **/current_pose:** 自車位置と姿勢
* **/detected_objects:** 感知されたオブジェクトの情報
* **/planned_path:** 計画された経路
* **/control_command:** 車両の制御コマンド
* **/safety_status:** システムの健全性に関する情報

**逸脱量**

AutowareのPlanningコンポーネントは、経路計画中に以下のような逸脱量を考慮します。

* velocity_deceleration:** 速度逸脱量
* acceleration_deceleration:** 加速度逸脱量
* jerk_deceleration:** ジャーク逸脱量

**'post resampling'**

Planningコンポーネントは、Planningされた経路を'post resampling'することにより、渋滞や障害物の存在に応じて経路を動的に調整します。

**追加情報**

Autowareの詳細については、以下のリソースを参照してください。

* [Autoware公式サイト](https://www.autoware.org)
* [Autowareドキュメント](https://docs.autoware.org/)

| 名前               | 型                                    | 説明                                                     |
| -----------------  | ------------------------------------- | ---------------------------------------------------------- |
| `output/odom`      | `nav_msgs::msg::Odometry`            | 縦方向とヨー方向のツイストが抑制されたオドメトリ         |
| `debug/stop_flag`  | `tier4_debug_msgs::msg::BoolStamped` | 車両が停止しているかどうかを示すフラグ                  |

## パラメータ

{{ json_to_markdown("localization/autoware_stop_filter/schema/stop_filter.schema.json") }}


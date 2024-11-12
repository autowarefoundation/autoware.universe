# Planning コンポーネント

## 概要

Autoware.Universe Planning モジュールは、広範なオープンソース自動運転ソフトウェアスタックにおける最先端のコンポーネントです。これらのモジュールは自動車両のナビゲーションにおいて重要な役割を果たし、ルートプランニング、動的障害物回避、さまざまな交通状況へのリアルタイム適応を巧みに処理します。

- Planning コンポーネントの高レベル概念については、[Planning コンポーネント設計ドキュメント](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/)を参照してください。
- Planning コンポーネントが他のコンポーネントとどのように連携するかを理解するには、[Planning コンポーネントのインターフェイスドキュメント](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/planning/)を参照してください。
- [ノードダイアグラム](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/)は、Planning モジュールを含む Autoware.Universe のすべてのモジュールの相互作用、入力、出力を図示しています。

## Planning モジュール

Planning コンポーネントの **モジュール** は、ソフトウェアのプランニングシステムを共同で形成するさまざまなコンポーネントを指します。これらのモジュールは、自動車両のプランニングに必要なさまざまな機能をカバーしています。Autoware の Planning モジュールはモジュール化されており、ユーザーは構成を変更することでどの機能を有効にするかをカスタマイズできます。このモジュール設計により、自動車両での運用におけるさまざまなシナリオと要件に柔軟に適応できます。

### Planning モジュールの有効化または無効化

モジュールの有効化と無効化には、キー構成ファイルと起動ファイルの設定を管理することが必要です。

### 構成のキーファイル

`default_preset.yaml`ファイルはプライマリ構成ファイルとして機能し、そこで Planning モジュールを無効化または有効化できます。さらに、ユーザーはさまざまなモーションプランナー間でモーションプランナーの種類を設定することもできます。たとえば:

- `launch_avoidance_module`: Avoidance モジュールを有効にする場合は `true` に設定し、無効にする場合は `false` に設定します。
- `motion\_stop\_planner\_type`: `default` を `obstacle\_stop\_planner` または `obstacle\_cruise\_planner` のいずれかに設定します。

!!! note

    `default_preset.yaml` を表示するには [ここ](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/preset/default_preset.yaml)をクリックします。

[起動ファイル](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/launch/scenario_planning/lane_driving) は、`default_preset.yaml` で定義された設定を参照して、動作経路プランナーのノードが実行されているときに構成を適用します。


```xml
<param name="avoidance.enable_module" value="$(var launch_avoidance_module)"/>
```

launch_avoidance_module から `default_preset.yaml` に対応。

### パラメータ設定

設定可能なパラメータが数多く用意されており、ユーザーは [こちら](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning) で変更できます。すべてのモジュールが `rqt_reconfigure` を介して調整できるわけではないことに注意してください。変更を有効にするには、パラメータを変更してから Autoware を再起動します。さらに、各パラメータの詳細情報は、プランニングタブの下にある対応するドキュメントに記載されています。

### Autoware へのカスタムモジュールの統合: 段階的なガイド

このガイドでは、カスタムモジュールを Autoware に統合する手順について説明します。

- デフォルトパラメータ `default_preset.yaml` ファイルにモジュールを追加します。例:


```yaml
- arg:
  name: launch_intersection_module
  default: "true"
```

- モジュールを [launcher](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/launch/scenario_planning) に統合します。例: [behavior_planning.launch.xml](https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml)


```xml
<arg name="launch_intersection_module" default="true"/>

<let
  name="behavior_velocity_planner_launch_modules"
  value="$(eval &quot;'$(var behavior_velocity_planner_launch_modules)' + 'behavior_velocity_planner::IntersectionModulePlugin, '&quot;)"
  if="$(var launch_intersection_module)"
/>
```

- 適用可能な場合、パラメータフォルダを適切な既存のパラメータフォルダ内に配置します。たとえば、次のような[交差点モジュールのパラメータ](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/intersection.param.yaml)は[behavior_velocity_planner](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner)内にあります。
- [tier4_planning_component.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/tier4_planning_component.launch.xml)に、パラメータのパスを挿入します。たとえば、次のような`behavior_velocity_planner_intersection_module_param_path`を使用します。


```xml
<arg name="behavior_velocity_planner_intersection_module_param_path" value="$(var behavior_velocity_config_path)/intersection.param.yaml"/>
```

- 対応するランチャー内でパラメータパスの変数を定義します。たとえば、[behavior_planning.launch.xml](https://github.com/autowarefoundation/autoware.universe/blob/04aa54bf5fb0c88e70198ca74b9ac343cc3457bf/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml#L191)


```xml
<param from="$(var behavior_velocity_planner_intersection_module_param_path)"/>
```

!!! 注意

    追加したい特定のモジュールに応じて、関連ファイルと手順が異なる場合があります。このガイドは、一般的な概要を提供し、出発点として機能します。これらの手順をモジュールの詳細に合わせて調整することが重要です。

## コミュニティ主導の取り組みに参加する

Autoware はコミュニティコラボレーションを重視しています。大小を問わず、すべての貢献は私たちにとって貴重です。バグの報告、改善の提案、新しいアイデアの提供、その他考えられることなど、すべてを歓迎します。

### 貢献の仕方

貢献する準備はできていますか？素晴らしい！まず、[貢献ガイドライン](https://autowarefoundation.github.io/autoware-documentation/main/contributing/) にアクセスして、参加に必要なすべての情報を入手してください。これには、バグレポートの提出、機能強化の提案、コードベースへの貢献に関する手順が含まれます。

### Planning & Control ワーキンググループミーティングに参加する

Planning & Control ワーキンググループは、コミュニティの不可欠な部分です。私たちは 2 週間ごとに会合して、現在の進捗状況、今後の課題について話し合い、新しいアイデアについてブレインストーミングを行います。これらのミーティングは、私たちの議論や意思決定プロセスに直接貢献する素晴らしい機会です。

ミーティングの詳細：

- **頻度:** 2 週間ごと
- **曜日:** 木曜日
- **時間:** 午前 8 時 UTC（午後 5 時 JST）
- **議題:** 現在の進捗状況を議論し、今後の開発を計画します。過去のミーティングの議事録は [こちら](https://github.com/orgs/autowarefoundation/discussions?discussions_q=is%3Aopen+label%3Ameeting%3Aplanning-control-wg+) で確認できます。

私たちのミーティングに参加することに興味がありますか？ぜひ参加してください！参加方法の詳細については、次のリンクをご覧ください。[ワーキンググループに参加する方法](https://github.com/autowarefoundation/autoware-projects/wiki/Autoware-Planning-Control-Working-Group#how-to-participate-in-the-working-group)。

### 引用

時々、Autoware の Planning Component に特化した論文を公開しています。これらの出版物を閲覧し、あなたの仕事に役立つ貴重な洞察を得ることをお勧めします。それらが役に立ち、プロジェクトで私たちの方法論やアルゴリズムの一部を組み込んだ場合、私たちの論文を引用していただけると非常に助かります。このサポートにより、より広い視聴者にリーチし、この分野への貢献を続けることができます。

Planning Component の [Motion Velocity Smoother](./autoware_velocity_smoother/README.md) モジュールでジャーク制約速度計画アルゴリズムを使用する場合は、関連する論文を引用していただけるようお願いします。

<!-- cspell:ignore Shimizu, Horibe, Watanabe, Kato -->

Y. 清水、T. 堀部、F. 渡辺、加藤正樹、"[自律移動体のジャーク制約速度計画: 線形計画法アプローチ](https://arxiv.org/abs/2202.10029)"、2022 年国際ロボット工学および自動化会議 (ICRA)


```tex
@inproceedings{shimizu2022,
  author={Shimizu, Yutaka and Horibe, Takamasa and Watanabe, Fumiya and Kato, Shinpei},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)},
  title={Jerk Constrained Velocity Planning for an Autonomous Vehicle: Linear Programming Approach},
  year={2022},
  pages={5814-5820},
  doi={10.1109/ICRA46639.2022.9812155}}
```


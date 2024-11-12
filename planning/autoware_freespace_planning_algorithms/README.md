# フリースペースのパスプランニングアルゴリズム

## 役割

このパッケージはフリースペースにおけるパスプランニングアルゴリズムの開発用です。

### 実装されたアルゴリズム

- Hybrid A\*とRRT\*(RRTとinformed RRT\*を含む)

informed-RRT\*の実装に関する詳細は、[rrtstar.md](rrtstar.md)を参照してください。

<!-- cspell:ignore Gammell -->

注: RRT\*については、RRT\*で実行可能な解が見つかった後に更新するかどうかを選択できます。
そうしない場合、アルゴリズムはバニラRRTと同じになります（リワイヤリング手順は違います）。
更新する場合は、実行可能な解が見つかった後のサンプリングが「informed」であるかどうかを選択できます。
これを真に設定すると、アルゴリズムは「2014年のGammellらによるinformed RRT\*」と同じになります。

## アルゴリズムの選択

アルゴリズムの速度と結果の解の品質にはトレードオフがあります。
アルゴリズムを(高品質解/低速)から(低品質解/高速)の範囲で並べると、次のようになります。
A\* -> informed RRT\* -> RRT。ほとんどの場合において、informed RRT\*は、同じ計算時間バジェットでRRT\*よりも解の品質が優れています。そのため、RRT\*は比較で省略されています。

選択基準は次のとおりです。

- 障害物の形状が複雑な場合: -> RRTとRRT\*を回避します。結果のパスが乱雑になる可能性があります。
- 目標位置がスタートから遠い場合: -> A\*を回避します。グリッドの離散化に基づいているため、時間がかかりすぎます。

## 新規アルゴリズムの実装ガイド

- このパッケージ内のすべてのプランニングアルゴリズムクラスは、`AbstractPlanningAlgorithm`クラスを継承する必要があります。必要に応じて、仮想関数をオーバーライドしてください。
- すべてのアルゴリズムは`nav_msgs::OccupancyGrid`タイプのコストマップを使用する必要があります。
したがって、`AbstractPlanningAlgorithm`クラスは主に、コストマップを使用した衝突チェック、グリッドベースのインデックス作成、コストマップに関連する座標変換を実装します。
- すべてのアルゴリズムは、`PlannerCommonParam`タイプの構造とアルゴリズム固有タイプの構造の両方をコンストラクタの入力として受け取る必要があります。たとえば、`AstarSearch`クラスのコンストラクタは、`PlannerCommonParam`と`AstarParam`の両方を受け取ります。

## スタンドアロンテストと可視化の実行

パッケージをros-testでビルドし、テストを実行します。


```sh
colcon build --packages-select autoware_freespace_planning_algorithms
colcon test --packages-select autoware_freespace_planning_algorithms
```

テストでは、シミュレーション結果は `/tmp/fpalgos-{アルゴリズムタイプ}-case{シナリオ番号}` に Rosbag として格納されます。
[test/debug_plot.py](test/debug_plot.py) を使用してこれらの結果ファイルをロードすることで、以下に示す図のように、経路と障害物を視覚化するプロットを作成できます。作成された図は `/tmp` に再度保存されます。

### A\* (単曲率ケース)

![サンプル出力図](figs/summary-astar_single.png)

### 200msec の時間予算のある情報ベース RRT\*

![サンプル出力図](figs/summary-rrtstar_informed_update.png)

### 更新のない RRT\* (RRT とほぼ同じ)

![サンプル出力図](figs/summary-rrtstar_fastest.png)

それぞれ、黒いセル、緑色のボックス、赤色のボックスは、障害物、開始コンフィギュレーション、目標コンフィギュレーションを示しています。
青いボックスのシーケンスはソリューションパスを示しています。

## Python モジュールへの拡張（A\* のみサポート）

Python モジュールへの拡張の実装があります。
以下を設定することで、Python 経由で A\* 検索を試すことができます:

- パラメーター
- コストマップ
- 自車位置
- ゴール位置

すると、次のものを入手できます:

- 成功または失敗
- 探索された軌跡

サンプルコードは [scripts/example/example.py](scripts/example/example.py) です。
このパッケージを事前にビルドして、セットアップシェルのスクリプトをソースする必要があることに注意してください。

## ライセンスの通知

ファイル `src/reeds_shepp.cpp` および `include/astar_search/reeds_shepp.h`
は [pyReedsShepp](https://github.com/ghliu/pyReedsShepp) から取得されています。
`pyReedsShepp` の実装も [ompl](https://github.com/ompl/ompl) のコードを大いに基にしていることに注意してください。
`pyReedsShepp` と `ompl` はどちらも 3 項 BSD ライセンスで配布されています。


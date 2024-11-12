## 回避モジュール（動的物体用）

本モジュールは開発中です。

## 目的 / 役割

本モジュールは、自身経路の周辺にある車両、歩行者、および障害物を回避する機能を [autoware_path_optimizer](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_path_optimizer/) と併用して提供します。
各モジュールは以下のような役割を果たします。
動的回避モジュールは回避対象の目標位置と速度に基づき、走行可能領域を切り取ります。
障害物回避モジュールは送信された走行可能領域の範囲に収まるように、走行経路を修正します。

[静的回避モジュール](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_behavior_path_static_obstacle_avoidance_module/) によって、静的障害物回避機能も提供されますが、これらのモジュールは役割が異なります。
静的障害物回避モジュールは、自身の車線の外側を介して回避を実行しますが、移動中の物体を回避することはできません。
一方、このモジュールは移動中の物体を回避できます。
そのため、本モジュールの名称には「動的」という言葉が使用されています。
以下の表に、各状況に対応する回避モジュールを記載します。

|                          |                         車線内での回避                         | 車線外の外側からの回避 |
| :----------------------- | :------------------------------------------------------------------------: | :------------------------------------: |
| 未稼働オブジェクトの回避 | Avoidance Module <br> Dynamic Avoidance Module + Obstacle Avoidance Module |            Avoidance Module            |
| 稼働オブジェクトの回避 |            Dynamic Avoidance Module + Obstacle Avoidance Module            |     モジュールなし (開発中)      |

## アルゴリズムのポリシー

ここでは、内部アルゴリズムのポリシーについて説明します。
内部アルゴリズムは 2 つの部分に分けることができます。1 つ目は障害物を回避するかどうかを判断し、2 つ目は対応する障害物に対する走行可能領域を切り取ります。

### 回避する障害物の選択

オブジェクトを回避するかを判断するためには、予測されたパスと各オブジェクトの状態（ポーズとツイスト）の両方が使用されます。
このモジュールに回避してほしいオブジェクトの種類も必要です。
この情報を使用して、モジュールは自車の進行を妨げ、回避可能なオブジェクトを回避することを決定します。

自車の進行を妨げるという定義は、数秒以内に衝突するオブジェクトとして実装されています。
もう 1 つ、「回避可能」は、乗客や他の車両にリスクを与えることなく回避できるかどうかを示します。
この目的のために、モジュールは障害物が横加速度と横ジャークの制約を満たして回避できるかどうかを判断します。
例えば、横方向に近すぎるか速すぎるオブジェクトは回避しないことを決定します。

### 選択された車両に対する走行可能領域の切り取り

回避するために選択された障害物に対して、モジュールは走行可能領域を切り取ります。
切り取りポリゴンの形状を決定するための入力として、障害物のポーズが主に使用されます。ただし、予測されたパスではなく、それらが自車のパスと並行に移動すると想定されます。
この設計は、オブジェクトの予測されたパスは（少なくとも現在は）パス変更を使用するほど正確ではないというところから来ています。
さらに、出力の走行可能領域の形状は、計算を平面ではなくスカラーで行うために、自車のパスに沿った長方形の切り取りとして設計されています。

#### 横方向の寸法の決定

ポリゴンの横方向の寸法は次のように計算されます。
走行可能領域から抽出するポリゴンの幅は、障害物の幅と `drivable_area_generation.lat_offset_from_obstacle` です。
`drivable_area_generation.max_lat_offset_to_avoid` によって横方向のシフトの長さを制限できます。

![drivable_area_extraction_width](./image/drivable_area_extraction_width.drawio.svg)

#### 縦方向の寸法の決定

次に、走行可能領域から同じ方向と反対方向の障害物を取り出すのは、TTC（衝突時間）を考慮して次のように機能します。

同じ方向の障害物に関して、TTC が負の障害物は無視されます（例: 障害物が自車の前にあり、障害物の速度が自車の速度よりも大きい）。

**同じ方向の障害物**（実装によってパラメータ名が異なる場合があります）
![same_directional_object](./image/same_directional_object.svg)

**反対方向の障害物**（実装によってパラメータ名が異なる場合があります）
![opposite_directional_object](./image/opposite_directional_object.svg)

### 選択された歩行者に対する走行可能領域の切り取り

次に、回避すべき歩行者に走行可能領域を生成するロジックについて説明します。
このタイプのオブジェクトは、自車の安全を確保しつつ、自車に対して優先権を持つものとみなされます。
言い換えれば、モジュールは次図に示すように、特定の時間の予測されたパスに基づいて特定の確信度で、特定の余裕を持った障害物に走行可能領域を割り当てます。

<figure>
    <img src="./image/2024-04-18_15-13-01.png" width="600">
    <figcaption>制限領域は歩行者の予測されたパスから生成されます</figcaption>
</figure>

モジュールは、オブジェクト用のポリゴンとは別に、自車の安全性を確保するための別のポリゴンも生成します。つまり、急な操舵またはパスからの大幅な変化を回避します。
これは、車両に対する回避動作と似ており、回避すべきオブジェクトとの安全距離の確保よりも優先されます。
その結果、下の図に示すように、オブジェクトの周りのポリゴンが自車の安全なポリゴンによって縮小されたものが自車の走行可能領域から引き算されます。

<figure>

## 図

<figcaption>自車の最小要求を対象物軌跡との余裕で優先する</figcaption>

## 例

<figure>
    <img src="./image/image-20230807-151945.png" width="800">
    <figcaption>バスの出発に対する回避</figcaption>
</figure>

<figure>  
    <img src="./image/image-20230807-152835.png" width="800">
    <figcaption>曲線上の回避</figcaption>
</figure>

<figure>
    <img src="./image/image-20230808-095936.png" width="800">
    <figcaption>対向車に対する回避</figcaption>
</figure>

<figure>
    <img src="./image/image-20230808-152853.png" width="800">
    <figcaption>複数の車に対する回避</figcaption>
</figure>

## 今後の課題

現在、経路シフト長は `drivable_area_generation.max_lat_offset_to_avoid` によって0.5メートル以内に制限されています。
これは、他のモジュールやPlanningコンポーネントの構造と連携する機能がないことが原因です。
この問題により、このモジュールは回避幅が小さい状況でのみ処理できます。
この問題は、このモジュールにとって最も重要です。
また、このモジュールが必要に応じて走行可能領域を拡張する能力も必要です。

## パラメーター

開発中

| 名称                                                              | 単位 | 型    | 説明                                                              | デフォルト値 |
| :--------------------------------------------------------------- | :---- | :----- | :------------------------------------------------------------------ | :------------ |
| `target_object.car`                                             | [-]   | bool   | 車の回避フラグ                                                          | true          |
| `target_object.truck`                                            | [-]   | bool   | トラックの回避フラグ                                                        | true          |
| ...                                                            | [-]   | bool   | ...                                                                   | ...           |
| `target_object.min_obstacle_vel`                                | [m/s] | double | 回避する際の最小障害物速度                                               | 1.0           |
| `drivable_area_generation.lat_offset_from_obstacle`              | [m]   | double | 障害物からの回避用横方向オフセット                                       | 0.8           |
| `drivable_area_generation.max_lat_offset_to_avoid`              | [m]   | double | 回避する際の最大横方向オフセット                                         | 0.5           |
| `drivable_area_generation.overtaking_object.max_time_to_collision` | [s]   | double | タイムトゥーコリジョンを計算する際の最大値                               | 3.0           |
| `drivable_area_generation.overtaking_object.start_duration_to_avoid` | [s]   | double | 障害物を通過する前に回避を考慮する期間                              | 4.0           |
| `drivable_area_generation.overtaking_object.end_duration_to_avoid` | [s]   | double | 障害物を通過した後に回避を考慮する期間                            | 5.0           |
| `drivable_area_generation.overtaking_object.duration_to_hold_avoidance` | [s]   | double | 障害物を通過した後に回避を保持する期間                            | 3.0           |
| `drivable_area_generation.oncoming_object.max_time_to_collision`   | [s]   | double | タイムトゥーコリジョンを計算する際の最大値                               | 3.0           |
| `drivable_area_generation.oncoming_object.start_duration_to_avoid` | [s]   | double | 障害物を通過する前に回避を考慮する期間                              | 9.0           |
| `drivable_area_generation.oncoming_object.end_duration_to_avoid`   | [s]   | double | 障害物を通過した後に回避を考慮する期間                            | 0.0           |


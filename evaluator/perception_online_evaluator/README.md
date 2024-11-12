# Perception Evaluator

知覚システムの出力評価用ノードです。

## 目的

このモジュールでは、アノテーションなしで知覚結果がどれだけ正確に生成されているかを評価できます。性能を確認でき、数秒前の結果を評価してオンライン実行を可能にします。

## 仕組み / アルゴリズム

評価されるメトリクスは次のとおりです。

- predicted_path_deviation
- predicted_path_deviation_variance
- lateral_deviation
- yaw_deviation
- yaw_rate
- total_objects_count
- average_objects_count
- interval_objects_count

### predicted_path_deviation / predicted_path_deviation_variance

過去のオブジェクトの予測経路と実際の移動経路を比較して、**移動中のオブジェクト**の逸脱量を算出します。各オブジェクトについて、予測経路のポイントと実際の経路の対応するポイント間の平均距離を、指定された時間ステップまで計算します。言い換えると、平均変位誤差（ADE）を計算します。評価対象のオブジェクトは $T_N$ 秒前のオブジェクトで、$T_N$ は予測時間幅 $[T_1, T_2, ..., T_N]$ の最大値です。

> [!NOTE]
> $T_N$ 秒前のオブジェクトは、すべてのメトリクスで対象オブジェクトです。これは、メトリクス全体で対象オブジェクトの時間を統一するためです。

![path_deviation_each_object](./images/path_deviation_each_object.drawio.svg)

$$
\begin{align}
n_{points} = T / dt \\
ADE = \Sigma_{i=1}^{n_{points}} d_i / n_{points}　\\
Var = \Sigma_{i=1}^{n_{points}} (d_i - ADE)^2 / n_{points}
\end{align}
$$

- $n_{points}$ : 予測経路のポイント数
- $T$ : 予測評価の時間幅。
- $dt$ : 予測経路の時間間隔
- $d_i$ : 経路ポイント $i$ での予測経路と実際の移動経路との距離
- $ADE$ : 対象オブジェクトの予測経路の平均逸脱量。
- $Var$ : 対象オブジェクトの予測経路逸脱量の分散。

最終的な予測経路逸脱量のメトリクスは、同じクラスのすべてのオブジェクトの予測経路の平均逸脱量を平均して計算した後、その平均逸脱量の平均、最大、最小値を計算します。

![path_deviation](./images/path_deviation.drawio.svg)

$$
\begin{align}
ADE_{mean} = \Sigma_{j=1}^{n_{objects}} ADE_j / n_{objects} \\
ADE_{max} = max(ADE_j) \\
ADE_{min} = min(ADE_j)
\end{align}
$$

Var_{max} = max(Var_j) \\
Var_{min} = min(Var_j)
\end{align}
$$

- $n_{objects}$: オブジェクト数
- $ADE_{mean}$: すべてのオブジェクトを通じた予測パスの平均偏差
- $ADE_{max}$: すべてのオブジェクトを通じた予測パスの最大偏差
- $ADE_{min}$: すべてのオブジェクトを通じた予測パスの最小偏差
- $Var_{mean}$: すべてのオブジェクトを通じた予測パス偏差の平均分散
- $Var_{max}$: すべてのオブジェクトを通じた予測パス偏差の最大分散
- $Var_{min}$: すべてのオブジェクトを通じた予測パス偏差の最小分散

実際の指標名は、オブジェクトクラスと時間視野によって決まります。たとえば、`predicted_path_deviation_variance_CAR_5.00`

### 横偏差

**移動オブジェクト**の安定性を評価するために、滑らかな走行軌跡と認識された位置の横偏差を計算します。滑らかな走行軌跡は、パラメータ`smoothing_window_size`で指定されたウィンドウサイズを持つ中央移動平均フィルタをかけることで計算されます。横偏差は、`T`秒前のタイムスタンプを持つ過去のオブジェクトの認識された位置と、滑らかな走行軌跡を比較することで計算されます。停止しているオブジェクトでは、滑らかな走行軌跡が不安定なため、この指標は計算されません。

![lateral_deviation](./images/lateral_deviation.drawio.svg)

### ヨー偏差

**移動オブジェクト**の過去のオブジェクトの認識されたヨー角と、滑らかな走行軌跡のヨー方位角の偏差を計算します。滑らかな走行軌跡は、パラメータ`smoothing_window_size`で指定されたウィンドウサイズを持つ中央移動平均フィルタをかけることで計算されます。ヨー偏差は、`T`秒前のタイムスタンプを持つ過去のオブジェクトの認識された向きと滑らかな走行軌跡のヨー方位角を比較することで計算されます。
停止しているオブジェクトでは、滑らかな走行軌跡が不安定なため、この指標は計算されません。

![yaw_deviation](./images/yaw_deviation.drawio.svg)

### ヨーレート

前回のタイムステップからのヨー角の変化に基づいて、オブジェクトのヨーレートを計算します。**静止オブジェクト**で評価され、ヨーレート認識の安定性を評価します。ヨーレートは、過去のオブジェクトのヨー角と、前回のサイクルで受信されたオブジェクトのヨー角を比較することで計算されます。ここで、t2は`T_n`秒前のタイムスタンプです。

![yaw_rate](./images/yaw_rate.drawio.svg)

### オブジェクトカウント

指定された検出範囲内の各オブジェクトクラスの検出数をカウントします。これらの指標は最新のオブジェクトに対して測定され、過去のオブジェクトではありません。

![detection_counts](./images/detection_counts.drawio.svg)

図では、範囲`R`は半径のリスト（例：`r_1, r_2, ...`）と高さのリスト（例：`h_1, h_2, ...`）の組み合わせによって決まります。
たとえば、

- 範囲の`R = (r_1, h_1)`のCARの数は1
- 範囲の`R = (r_1, h_2)`のCARの数は2
- 範囲の`R = (r_2, h_1)`のCARの数は3
- 範囲の`R = (r_2, h_2)`のCARの数は4

#### 全オブジェクトカウント

指定された検出範囲内の各クラスの一意のオブジェクトの数をカウントします。全オブジェクトカウントは次のように計算されます。

$$
\begin{align}
\text{全オブジェクトカウント (クラス、範囲)} & = \left| \bigcup_{t=0}^{T_{\text{now}}} \{ \text{uuid} \mid \text{class}(t, \text{uuid}) = C \wedge \text{position}(t, \text{uuid}) \in R \} \right|
\end{align}
$$

ここで、

- $\bigcup$ は、$t = 0$ から $T_{\text{now}}$ までのすべてのフレームにおける結合を表し、各 uuid が一度のみカウントされるようにします。
- $\text{class}(t, \text{uuid}) = C$ は、時刻 $t$ で uuid を持つオブジェクトがクラス $C$ に属することを示します。
- $\text{position}(t, \text{uuid}) \in R$ は、時刻 $t$ に uuid を持つオブジェクトが指定範囲 $R$ 内にあることを示します。
- $\left| \{ \ldots \} \right|$ は集合の濃度を示し、すべての考慮対象時間でクラスと範囲の基準を満たすすべての固有 uuid の数をカウントします。

#### 平均オブジェクト数

指定された検出範囲内の各クラスのオブジェクトの平均数をカウントします。この指標は、uuid を考慮せずに 1 フレームで検出されたオブジェクトの数を測定します。平均オブジェクト数は次のように計算されます。

$$
\begin{align}
\text{平均オブジェクト数(クラス、範囲)} = \frac{1}{N} \sum_{t=0}^{T_{\text{now}}} \left| \{ \text{object} \mid \text{class}(t, \text{object}) = C \wedge \text{position}(t, \text{object}) \in R \} \right|
\end{align}
$$

ここで、

- $N$ は時間間隔 $T\_{\text{now}}$ までのフレームの総数を表します（正確には `detection_count_purge_seconds`）。
- $text{object}$ は、時刻 $t$ でクラスと範囲の基準を満たすオブジェクトの数を表します。

#### 区間オブジェクト数

最後の `objects_count_window_seconds` において、指定された検出範囲内の各クラスのオブジェクトの平均数をカウントします。この指標は、uuid を考慮せずに 1 フレームで検出されたオブジェクトの数を測定します。区間オブジェクト数は次のように計算されます。

$$
\begin{align}
\text{区間オブジェクト数(クラス、範囲)} = \frac{1}{W} \sum_{t=T_{\text{now}} - T_W}^{T_{\text{now}}} \left| \{ \text{object} \mid \text{class}(t, \text{object}) = C \wedge \text{position}(t, \text{object}) \in R \} \right|
\end{align}
$$

ここで、

- $W$ は最後の `objects_count_window_seconds` 内のフレームの総数を表します。
- $T_W$ は時間窓 `objects_count_window_seconds` を表します。

## 入出力

| 名前              | タイプ                                              | 説明                                            |
| ----------------- | ------------------------------------------------- | ----------------------------------------------- |
| `~/input/objects` | `autoware_perception_msgs::msg::PredictedObjects` | 評価する予測オブジェクト                         |
| `~/metrics`       | `diagnostic_msgs::msg::DiagnosticArray`           | 知覚精度の診断情報                            |
| `~/markers`       | `visualization_msgs::msg::MarkerArray`            | デバッグと視覚化のためのビジュアルマーカー      |

## パラメータ

| 名称                                                    | 型         | 説明                                                                                                                                      |
| ------------------------------------------------------ | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `selected_metrics`                                     | リスト         | 横逸脱量、ヨー逸脱量、予測経路逸脱量などの評価する指標。                                                                                   |
| `smoothing_window_size`                                | 整数      | 経路の平滑化のためのウィンドウサイズを決定し、奇数にする必要があります。                                                                  |
| `prediction_time_horizons`                             | doubleリスト | 秒単位の予測評価のためのタイムホライゾン。                                                                                                 |
| `stopped_velocity_threshold`                           | double       | 車両の停止を確認するためのしきい値速度。                                                                                                  |
| `detection_radius_list`                                | doubleリスト | 評価対象のオブジェクトの検出半径。（オブジェクトカウント専用）                                                                               |
| `detection_height_list`                                | doubleリスト | 評価対象のオブジェクトの検出高さ。（オブジェクトカウント専用）                                                                                |
| `detection_count_purge_seconds`                        | double       | オブジェクト検出カウントを削除するためのタイムウィンドウ。                                                                                   |
| `objects_count_window_seconds`                         | double       | オブジェクト検出カウントを保持するためのタイムウィンドウ。このタイムウィンドウ内のオブジェクト検出数は `detection_count_vector_` に保存されます。 |
| `target_object.*.check_lateral_deviation`              | ブール値       | 特定のオブジェクトの種類（車、トラックなど）の横逸脱を確認するかどうか。                                                                   |
| `target_object.*.check_yaw_deviation`                  | ブール値       | 特定のオブジェクトの種類（車、トラックなど）のヨー逸脱を確認するかどうか。                                                                 |
| `target_object.*.check_predicted_path_deviation`       | ブール値       | 特定のオブジェクトの種類（車、トラックなど）の予測経路逸脱を確認するかどうか。                                                            |
| `target_object.*.check_yaw_rate`                       | ブール値       | 特定のオブジェクトの種類（車、トラックなど）のヨーレートを確認するかどうか。                                                               |
| `target_object.*.check_total_objects_count`            | ブール値       | 特定のオブジェクトの種類（車、トラックなど）の合計オブジェクト数をチェックするかどうか。                                                    |
| `target_object.*.check_average_objects_count`          | ブール値       | 特定のオブジェクトの種類（車、トラックなど）の平均オブジェクト数をチェックするかどうか。                                                  |
| `target_object.*.check_interval_average_objects_count` | ブール値       | 特定のオブジェクトの種類（車、トラックなど）の区間平均オブジェクト数をチェックするかどうか。                                               |
| `debug_marker.*`                                       | ブール値       | マーカー可視化（履歴パス、予測パスなど）のデバッグパラメーター。                                                                             |

## 仮定 / 既知の制限

予測オブジェクトの現在の位置は、おおむね正確であると想定されています。

## 将来の拡張 / 未実装部分

- クラスごとの認識率を向上
- 異常な物理的挙動を示すオブジェクトのメトリクス（例：フェンスを突き抜ける）
- オブジェクトの分割に対するメトリクス
- 通常は静止しているが移動するオブジェクトに対するメトリクス
- 消滅したオブジェクトのメトリクス


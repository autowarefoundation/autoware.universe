## autoware_pose_instability_detector

`pose_instability_detector`ノードは、拡張カルマンフィルタ（EKF）の出力トピックである`/localization/kinematic_state`の安定性を監視するために設計されています。

このノードは、次の2つのポーズ比較のために定期的なタイマーコールバックをトリガーします。

- `timer_period`秒前に取得した`/localization/kinematic_state`のポーズから始める、デッドレコニングによって計算されたポーズ
- `/localization/kinematic_state`からの最新のポーズ

この比較の結果は`/diagnostics`トピックに出力されます。

![overview](./media/pose_instability_detector_overview.png)

![rqt_runtime_monitor](./media/rqt_runtime_monitor.png)

このノードが`/diagnostics`にWARNメッセージを出力する場合、EKFの出力は統合されたツイスト値と大きく異なることを意味します。
言い換えると、WARN出力は、車両がツイスト値に基づいて予想される範囲外の位置に移動したことを示します。
この差異は、推定ポーズまたは入力ツイストのいずれかに問題があることを示唆しています。

次の図は、手順の概要を示しています。

![procedure](./media/pose_instabilty_detector_procedure.svg)

## デッドレコニングアルゴリズム

デッドレコニングは、前の位置と速度に基づいて車両の位置を推定する方法です。
デッドレコニングの手順は次のとおりです。

1. `/input/twist`トピックから必要なツイスト値を取得する。
2. ツイスト値を積分してポーズ遷移を計算する。
3. ポーズ遷移を前のポーズに適用して現在のポーズを取得する。

### ツイスト値の収集

`pose_instability_detector`ノードは、デッドレコニングを実行するために`~/input/twist`トピックからツイスト値を収集します。
理想的には、`pose_instability_detector`は前のポーズと現在のポーズの間のツイスト値が必要です。
したがって、`pose_instability_detector`はツイストバッファをスニップし、補間と外挿を適用して目的の時間でのツイスト値を取得します。

![how_to_snip_necessary_twist](./media/how_to_snip_twist.png)

### 線形遷移と角遷移

ツイスト値が収集されると、ノードはツイスト値に基づいて線形遷移と角遷移を計算し、それを前のポーズに追加します。

## 閾値の定義

`pose_instability_detector`ノードは、デッドレコニングで計算されたポーズとEKF出力からの最新のポーズを比較します。
これらの2つのポーズは理想的には同じですが、実際にはツイスト値のエラーとポーズ観測のために異なります。
これらの2つのポーズが絶対値が閾値を超えて大きく異なると、ノードは`/diagnostics`トピックにWARNメッセージを出力します。ポーズが大きく異なるかどうかを判断するための閾値（x、y、z、ロール、ピッチ、ヨー）は6つあり、これらの閾値は次のサブセクションによって決定されます。

### `diff_position_x`

この閾値は、2つのポーズ間の縦軸における差を調べ、車両が予想される誤差を超えていないかを確認します。
この閾値は、「速度スケールファクター誤差による最大縦断誤差」と「ポーズ推定誤差許容値」の合計です。

$$
\tau_x = v_{\rm max}\frac{\beta_v}{100} \Delta t + \epsilon_x\\
$$

| 記号        | 説明                                                                      | 単位  |
| ------------- | -------------------------------------------------------------------------------- | ----- |
| $\tau_x$      | 縦軸方向の差のしきい値                                                              | m   |
| $v_{\rm max}$ | 最大速度                                                                 | m/s |
| $\beta_v$     | 最大速度の許容スケールファクター                                            | $\%$  |
| $\Delta t$    | 時間間隔                                                                    | s   |
| $\epsilon_x$  | 姿勢推定器（例：ndt_scan_matcher）の縦軸方向の誤差許容値 | m   |

### `diff_position_y` および `diff_position_z`

これらのしきい値は、2つのポーズ間の横方向軸と鉛直方向軸の差を調べ、車両が想定される誤差を超えているかどうかを確認します。

`pose_instability_detector` は車両が走行する可能性のある範囲を計算し、通常のデッドレコニングポーズと限界ポーズの最大差を取得します。

![lateral_threshold_calculation](./media/lateral_threshold_calculation.png)

さらに、`pose_instability_detector` ノードはポーズ推定誤差許容値を考慮してしきい値を決定します。

$$
\tau_y = l + \epsilon_y
$$

| シンボル       | 説明                                                                                     | 単位 |
| ------------ | ----------------------------------------------------------------------------------------------- | ---- |
| $\tau_y$     | 横軸誤差の閾値                                                                             | $m$  |
| $l$          | 上記の画像で示される最大横方向距離 (計算方法は付録を参照)                                | $m$  |
| $\epsilon_y$ | ポーズ推定器 (例: ndt_scan_matcher) の横軸方向での許容誤差                                | $m$  |

`pose_instability_detector`では、y軸のしきい値はx軸と同じ値に設定されています。相違するのはpose estimatorの誤差許容度だけです。

### `diff_angle_x`, `diff_angle_y`, `diff_angle_z`

これらのしきい値は、2つの姿勢のロール角、ピッチ角、ヨー角の差を調べます。このしきい値は、「速度倍率係数の誤差による最大角度誤差とバイアス誤差」と「pose推定の誤差許容度」の合計です。

$$
\tau_\phi = \tau_\theta = \tau_\psi = \left(\omega_{\rm max}\frac{\beta_\omega}{100} + b \right) \Delta t + \epsilon_\psi
$$

| 記号 | 説明 | 単位 |
|---|---|---|
| $\tau_\phi$ | ロール角差のしきい値 | rad |
| $\tau_\theta$ | ピッチ角差のしきい値 | rad |
| $\tau_\psi$ | ヨー角差のしきい値 | rad |
| $\omega_{\rm max}$ | 最大角速度 | rad/s |
| $\beta_\omega$ | 最大角速度の許容スケール誤差 | % |
| $b$ | 角速度のバイアス許容値 | rad/s |
| $\Delta t$ | 時間間隔 | s |
| $\epsilon_\psi$ | 姿勢推定器（例：ndt_scan_matcher）のヨー角エラー許容値 | rad |

## パラメーター

{{ json_to_markdown("localization/autoware_pose_instability_detector/schema/pose_instability_detector.schema.json") }}

## 入力

| 名称                | 型                                             | 説明                |
| -------------------- | ----------------------------------------------- | --------------------- |
| `~/input/odometry`   | `nav_msgs::msg::Odometry`                       | EKFにより推定された姿勢 |
| `~/input/twist`      | `geometry_msgs::msg::TwistWithCovarianceStamped` | ひねりだけでなく速度も表す |

## 出力

このドキュメントは、私たちがAutoware上に構築した自動運転ソフトウェアの機能、アーキテクチャ、設計について説明します。

### 1. 概要

私たちのソフトウェアは、安全で効率的な自動運転を実現するために必要なすべてのモジュールを含んでいます。これらは、以下のコンポーネントで構成されています。

* **Planning (計画)**: パスを生成し、速度プロフィールを決定します。
* **Control (制御)**: 計画されたパスに従って車両を制御します。
* **Perception (認識)**: 周囲環境を認識し、障害物や道路標識を検出します。

### 2. アーキテクチャ

ソフトウェアはモジュール設計を採用しており、各モジュールが独立して機能します。このアーキテクチャにより、柔軟性と拡張性が向上しています。

モジュール間の通信はROS（Robot Operating System）を使用して行われます。ROSは、リアルタイムロボットアプリケーション向けのオープンソースミドルウェアです。

### 3. 設計

### 3.1 Planning

Planningモジュールは、DWA（Dynamic Window Approach）アルゴリズムを使用して、障害物を回避しながら目的地に向かう安全なパスを生成します。

### 3.2 Control

Controlモジュールは、車両の速度とステアリングを制御するために、PID（Proportional-Integral-Derivative）コントローラーを使用します。

### 3.3 Perception

Perceptionモジュールは、LIDARとカメラデータを使用して、障害物と道路標識を検出します。障害物の検出には、点群処理と物体検出アルゴリズムが使用されます。また、道路標識の検出には、画像処理と機械学習アルゴリズムが使用されます。

### 4. 実装

ソフトウェアはAutoware上に実装されています。Autowareは、自律走行車向けのオープンソースソフトウェアプラットフォームです。

### 5. 安全性機能

私たちのソフトウェアは、以下の安全性機能を備えています。

* **障害物検出**: 周囲の障害物を検出し、衝突を回避します。
* **速度制限**: 設定された速度制限を超えないように車両を制御します。
* **自己診断**: システムのコンポーネントを監視し、異常を検出します。

### 6. 性能

### 6.1 精度

Planningモジュールは、障害物を効果的に回避し、安全なパスを生成します。Controlモジュールは、車両を制御し、正確に計画されたパスに従います。

### 6.2 効率

私たちのソフトウェアは効率的に動作し、リアルタイムで実行できます。これにより、自動運転車両が迅速かつ確実に反応できます。

### 7. 課題

自動運転ソフトウェアの開発には、以下のような課題があります。

* **障害物の検出**: すべての種類の障害物を確実に検出することは困難な場合があります。
* **Planing**: 障害物が多い環境では、安全で効率的なパスを生成するのは困難な場合があります。
* **安全性**: 自動運転車両は、あらゆる状況で安全に行動する必要があります。

私たちは、これらの課題に取り組み、私たちのソフトウェアを継続的に改善しています。

| 名前                | タイプ                                  | 説明 |
| ------------------- | ------------------------------------- | ----------- |
| `~/debug/diff_pose` | geometry_msgs::msg::PoseStamped       | diff_pose   |
| `/diagnostics`      | diagnostic_msgs::msg::DiagnosticArray | Diagnostics |

## 付録

最大横距離 $l$ の計算では、`pose_instability_detector` ノードは次の姿勢を推定します。

| 自車位置                            | heading速度 $v$                             | angular速度 $\omega$                                      |
| ------------------------------- | ------------------------------------------------ | -------------------------------------------------------------- |
| Nominalデッドレコニング位置     | $v_{\rm max}$                                    | $\omega_{\rm max}$                                             |
| コーナーAのデッドレコニング位置 | $\left(1+\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1+\frac{\beta_\omega}{100}\right) \omega_{\rm max} + b$ |
| コーナーBのデッドレコニング位置 | $\left(1-\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1+\frac{\beta_\omega}{100}\right) \omega_{\rm max} + b$ |
| コーナーCのデッドレコニング位置 | $\left(1-\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1-\frac{\beta_\omega}{100}\right) \omega_{\rm max} - b$ |
| コーナーDのデッドレコニング位置 | $\left(1+\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1-\frac{\beta_\omega}{100}\right) \omega_{\rm max} - b$ |

進行方向速度 $v$ と $\omega$ が与えられた場合、前回のポーズから見た 2D 理論的な変化は次のように計算されます。

$$
\begin{align*}
\left[
    \begin{matrix}
    \Delta x\\
    \Delta y
    \end{matrix}
\right]
&=
\left[
    \begin{matrix}
    \int_{0}^{\Delta t} v \cos(\omega t) dt\\
    \int_{0}^{\Delta t} v \sin(\omega t) dt
    \end{matrix}
\right]
\\
&=
\left[
    \begin{matrix}
    \frac{v}{\omega} \sin(\omega \Delta t)\\
    \frac{v}{\omega} \left(1 - \cos(\omega \Delta t)\right)
    \end{matrix}
\right]
\end{align*}
$$

各コーナーについてこの変動を計算し、予測航法ポーズとコーナーポーズの距離を比較することで、横距離 $l$ の最大値を取得します。


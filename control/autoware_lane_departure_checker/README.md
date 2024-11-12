## 車線逸脱チェッカー

**車線逸脱チェッカー**は、車両が軌跡に従っているかどうかを確認します。軌跡に従っていない場合、`diagnostic_updater`を介してステータスを報告します。

## 機能

このパッケージには、次の機能が含まれています。

- **車線逸脱**: 制御モジュールからの出力（予測軌跡）に基づいて、自車位置が車線境界から逸脱するかどうかを確認します。
- **軌跡逸脱**: 自車位置が軌跡から逸脱していないかどうかを確認します。横方向、縦方向、偏揺角方向の逸脱を確認します。
- **路肩逸脱**: 制御の出力から生成された自車のフットプリントが路肩を越えて伸びているかどうかを確認します。

## 内部機能 / アルゴリズム

### 共分散によってフットプリントを拡張する方法

1. 車両座標系における誤差楕円（共分散）の標準偏差を計算します。

    1. 共分散を車両座標系に変換します。

    $$
    \begin{align}
    \left( \begin{array}{cc} x_{vehicle}\\ y_{vehicle}\\ \end{array} \right) = R_{map2vehicle}  \left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
    \end{align}
    $$

    車両座標系における共分散を計算します。

    $$
    \begin{align}
    Cov_{vehicle} &= E \left[
    \left( \begin{array}{cc} x_{vehicle}\\ y_{vehicle}\\ \end{array} \right) (x_{vehicle}, y_{vehicle}) \right] \\
    &= E \left[ R\left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
    (x_{map}, y_{map})R^t
    \right] \\
    &= R E\left[ \left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
    (x_{map}, y_{map})
    \right] R^t \\
    &= R Cov_{map} R^t
    \end{align}
    $$

    2. `post resampling`に展開する縦方向の長さは、$Cov_{vehicle}(0,0)$における$x_{vehicle}$の周辺分布に対応します。同様に、横方向の長さは$Cov_{vehicle}(1,1)$における$x_{vehicle}$の周辺分布に対応します。Wikipediaの参照 [こちら](https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Marginal_distributions)。

2. `footprint_margin_scale`を乗じた標準偏差に基づいてフットプリントを拡張します。

## インターフェイス

### 入力

- /localization/kinematic_state [`nav_msgs::msg::Odometry`]
- /map/vector_map [`autoware_map_msgs::msg::LaneletMapBin`]
- /planning/mission_planning/route [`autoware_planning_msgs::msg::LaneletRoute`]
- /planning/scenario_planning/trajectory [`autoware_planning_msgs::msg::Trajectory`]
- /control/trajectory_follower/predicted_trajectory [`autoware_planning_msgs::msg::Trajectory`]

### 出力

- [`diagnostic_updater`] lane_departure : 自車位置が車線から逸脱した場合に診断レベルを更新します。

## パラメータ

### ノードパラメータ

#### 一般パラメータ

| 名称                       | 型   | 説明                                                                                                 | デフォルト値 |
| :------------------------- | :----- | :---------------------------------------------------------------------------------------------------------- | :------------ |
| will_out_of_lane_checker   | bool   | 自車フットプリントが車線から逸脱するかどうかのチェッカーを有効にする                                 | True          |
| out_of_lane_checker        | bool   | 自車フットプリントが車線外にあるかどうかをチェッカーを有効にする                                      | True          |
| boundary_departure_checker | bool   | 自車フットプリントがboundary_types_to_detectで指定された境界から逸脱するかどうかのチェッカーを有効にする | False         |
| update_rate                | double | パブリッシュする頻度 [Hz]                                                                               | 10.0          |
| visualize_lanelet          | bool   | レーンレットを視覚化するフラグ                                                                           | False         |

#### 車線逸脱のためのパラメータ

| 名前 | タイプ | 説明 | デフォルト値 |
|---|---|---|---|
| `include_right_lanes` | ブール | 境界に右のレーンレットを含めるかどうか | `False` |
| `include_left_lanes` | ブール | 境界に左のレーンレットを含めるかどうか | `False` |
| `include_opposite_lanes` | ブール | 境界に対向のレーンレットを含めるかどうか | `False` |
| `include_conflicting_lanes` | ブール | 境界に交差するレーンレットを含めるかどうか | `False` |

#### 路側逸脱 パラメータ

* `# (m)`: 路側逸脱開始のしきい値 `'post resampling'` 距離（メートル）
* `# (rad)`: 路側逸脱開始のしきい値 `'post resampling'` 曲率（ラジアン）
* `# (m)`: `'post resampling'` 路側逸脱中の vehicle `'post resampling'` の速度（メートル）
* `# (s)`: `'post resampling'` 路側逸脱中の vehicle `'post resampling'` の経過時間（秒）
* `#`: `'post resampling'` 路側逸脱開始時の自車位置における heading 偏差（ラジアン）
* `# (m)`: `'post resampling'` 路側逸脱開始時の自車位置における lateral offset（メートル）
* `#`: `'post resampling'` 路側逸脱開始時の vehicle `'post resampling'` の速度（メートル/秒）
* `# (m^2/s)`: `'post resampling'` 路側逸脱中の vehicle `'post resampling'` の acceleration 逸脱量（平方メートル/秒）
* `# (m^2/s)`: `'post resampling'` 路側逸脱中の vehicle `'post resampling'` の velocity 逸脱量（平方メートル/秒）
* `# (m^2/s)`: 路側逸脱中の vehicle `'post resampling'` の heading 逸脱量（平方メートル/秒）
* `# (m)`: `'post resampling'` 路側逸脱終了時の vehicle `'post resampling'` の速度（メートル/秒）
* `#`: `'post resampling'` 路側逸脱終了時の自車位置における heading 偏差
* `# (s)`: `'post resampling'` 路側逸脱終了時の vehicle `'post resampling'` の経過時間
* `# (cm)`: 静止時の vehicle `'post resampling'` の steering wheel angle（センチメートル）
* `# (m)`: 静止時の vehicle `'post resampling'` の lateral offset（メートル）
* `#`: 静止時の vehicle `'post resampling'` の速度（メートル/秒）

| 名称                   | タイプ                         | 説明                                                      | デフォルト値 |
| :----------------------- | :--------------------------- | :--------------------------------------------------------- | :------------ |
| boundary_types_to_detect | std::vector\<std::string\> | boundary_departure_checkerで検出するline_stringタイプ | [road_border] |

### 主要パラメータ

| 名称                       | 種類   | 説明                                                                                                  | デフォルト値 |
| :------------------------- | :----- | :------------------------------------------------------------------------------------------------------- | :------------ |
| footprint_margin_scale     | 数値   | footprintマージンを拡張する係数。標準偏差に1を乗算                                         | 1.0           |
| footprint_extra_margin     | 数値   | footprintマージンを拡張する係数。 レーン逸脱のチェック時                                     | 0.0           |
| resample_interval          | 数値   | trajectoryを再サンプリングする際のポイント間の最小ユークリッド距離 (m) | 0.3           |
| max_deceleration           | 数値   | 制動距離を計算する際の最大減速度                                                         | 2.8           |
| delay_time                 | 数値   | 制動距離を計算する際のブレーキ作動までの遅延時間 (秒)                                  | 1.3           |
| max_lateral_deviation      | 数値   | 車両座標系における最大横方向逸脱距離 (m)                                                 | 2.0           |
| max_longitudinal_deviation | 数値   | 車両座標系における最大縦方向逸脱距離 (m)                                                 | 2.0           |
| max_yaw_deviation_deg      | 数値   | trajectoryからの自己車両の最大ヨー逸脱角度 (度)                                            | 60.0          |


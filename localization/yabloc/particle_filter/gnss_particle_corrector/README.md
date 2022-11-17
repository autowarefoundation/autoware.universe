# GNSS Particle Corrector

## Purpose

- GNSSを用いてパーティクルの重み付けを行うパッケージ。
- `ublox_msgs::msg::NavPVT` と `geometry_msgs::msg::PoseWithCovarianceStamped` の2種類の入力に対応している。

## Inputs / Outputs

### Input

| Name         | Type                      | Description           |
| ------------ | ------------------------- | --------------------- |
| `/predicted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | predicted particles |
| `/input/navpvt` | `ublox_msgs::msg::NavPVT` | position measurement |
| `/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | position measurement |
| `/height` | `std_msgs::msg::Float32` | ground height (optional) |

### Output

| Name              | Type                                 | Description                                              |
| ----------------- | ------------------------------------ | -------------------------------------------------------- |
| `/weighted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | 重み付けされたパーティクル |
| `/gnss/effect_marker` | `visualization_msgs::msg::MarkerArray` | GNSSによる重み付けの分布を同心円で可視化したもの |

## Parameters

| Name                     | Type    | Default | Description                                                                         |
| ------------------------ | ------- | ------- | ----------------------------------------------------------------------------------- |
| `likelihood_min_weight`  | double  | 0.01    |  パーティクルに付与する重みの最小値 |
| `likelihood_stddev`      | double  | 5.0     |  重みの分布の標準偏差 |
| `likelihood_flat_radius` | double  | 1.0     |  重みの分布の不感帯。観測位置との距離がこれより小さいなら重みは1になる |
| `rtk_enabled`            | bool    | true    | `NavPVT`を利用するときにのみ有効。`false`なら常に`FLOAT`としてみなされる。     |
| `flat_range_gain`        | double  | 5.0     | `NavPVT`を利用するときにのみ有効。RTKが`FLOAT`のときに`stddev`と`flat_radius`を何倍するか|

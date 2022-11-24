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

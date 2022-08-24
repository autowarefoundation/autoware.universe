# Modularized Particle Filter

## Purpose

- Particle Filterの機能を提供するパッケージ。
- Correctorは抽象的なクラスのみを提供しており、それを継承して好みのCorrectorノードを実装することを想定している。
- Predictorはそのまま使える。

## Predictor

### Input

| Name         | Type                      | Description           |
| ------------ | ------------------------- | --------------------- |
| `/pose` | `geometry_msgs::msg::Pose` | パーティクルの初期値位置指定用 |
| `/initialpose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | パーティクルの初期値位置指定用 |
| `/twist` | `geometry_msgs::msg::TwistStamped` | 予測更新の速度と角速度を受け取る |
| `/weighted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | 重み付けされたパーティクル |
| `/height` | `std_msgs::msg::Float32` | ground height (optional) パーティクルの高さを指定する |

### Output

| Name              | Type                                 | Description                                              |
| ----------------- | ------------------------------------ | -------------------------------------------------------- |
| `/predicted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | 予測更新されたパーティクル |
| `/particle_pose` | `geometry_msgs::msg::PoseStamped` | パーティクルの重みつき平均 |

### Parameters

| Name                     | Type    | Default | Description                                                                         |
| ------------------------ | ------- | ------- | ----------------------------------------------------------------------------------- |
| `prediction_rate`  | double| 50 | 予測更新をする頻度単位はHz |
| `visualize`  | bool | false   | パーティクルをvisualization_msgsでもpublishするか否か |
| `num_of_particles`      | int | 500     | パーティクルの総数 |
| `resampling_interval_seconds` | double  | 1.0     |  リサンプリングを実施する頻度 |
| `static_linear_covariance` | double | 0.01| `/twist`の分散を上書きする値。`/twist`のもとの分散値を使うオプションは無い|
| `static_angular_covariance`| double | 0.01| `/twist`の分散を上書きする値。`/twist`のもとの分散値を使うオプションは無い|
| `use_dynamic_noise`| bool | false | 低速域において分散を小さくするかモードを使うか否か|

## Corrector

### Corrector Input

| Name         | Type                      | Description           |
| ------------ | ------------------------- | --------------------- |
| `/predicted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | 予測更新されたパーティクル |

### Corrector Output

| Name              | Type                                 | Description                                              |
| ----------------- | ------------------------------------ | -------------------------------------------------------- |
| `/weighted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | 重み付けされたパーティクル |

### Corrector Parameters

| Name                     | Type    | Default | Description                                                                         |
| ------------------------ | ------- | ------- | ----------------------------------------------------------------------------------- |
| `/visualize`  | bool | false   | パーティクルをvisualization_msgsでもpublishするか否か |

## GNSS Corrector

### Input

| Name                   | Type                                                   | Description                               |
|------------------------|--------------------------------------------------------|-------------------------------------------|
| `/predicted_particles` | `yabloc_particle_filter::msg::ParticleArray` | particles predicted by the predictor node |

### Output

| Name                  | Type                                                   | Description                              |
|-----------------------|--------------------------------------------------------|------------------------------------------|
| `/weighted_particles` | `yabloc_particle_filter::msg::ParticleArray` | particles weighted by the corrector node |

### Parameters

| Name         | Type | Default | Description                                                       |
|--------------|------|---------|-------------------------------------------------------------------|
| `/visualize` | bool | false   | whether particles are also published in visualization_msgs or not |


# GNSS Particle Corrector

## Purpose

- Package for particle weighting using GNSS.
- It supports two types of input: `ublox_msgs::msg::NavPVT` and `geometry_msgs::msg::PoseWithCovarianceStamped`.

## Inputs / Outputs

### Input

| Name                    | Type                                                   | Description                              |
|-------------------------|--------------------------------------------------------|------------------------------------------|
| `/predicted_particles`  | `modularized_particle_filter_msgs::msg::ParticleArray` | predicted particles                      |
| `/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped`        | pose measurement for weighting           |
| `/input/navpvt`         | `ublox_msgs::msg::NavPVT`                              | GNSS measurement for weighting           |
| `/input/height`         | `std_msgs::msg::Float32`                               | ground height used for gnss_range_marker |

### Output

| Name                  | Type                                                   | Description                         |
|-----------------------|--------------------------------------------------------|-------------------------------------|
| `/weighted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | weighted particles                  |
| `/gnss_range_marker`  | `visualization_msgs::msg::MarkerArray`                 | visualized GNSS weight distribution |



### Parameters

| Name                              | Type  | Default | Description                                                                                           |
|-----------------------------------|-------|---------|-------------------------------------------------------------------------------------------------------|
| `/ignore_less_than_float`         | bool  | true    | if this is true, only FIX or FLOAT is used for correction (No effect when using pose_with_covariance) |
| `/mahalanobis_distance_threshold` | float | 20.f    | if the Mahalanobis distance to the GNSS for particle exceeds this, the correction skips.              |

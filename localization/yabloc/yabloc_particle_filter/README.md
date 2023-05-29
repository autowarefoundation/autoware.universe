# YabLoc Particle Filter

## Purpose

- This package provides the functionality of particle filter.
- A corrector provides only an abstract class and is intended to be inherited to implement a corrector node of your choice.
- A predictor can be used as is.

## How to test

```shell
colcon test --event-handlers console_cohesion+  --packages-select yabloc_particle_filter
colcon test-result --verbose --all
```


## Predictor node

### Input

| Name                  | Type                                                   | Description                                               |
|-----------------------|--------------------------------------------------------|-----------------------------------------------------------|
| `/initialpose`        | `geometry_msgs::msg::PoseWithCovarianceStamped`        | to specity the initial position of particles              |
| `/twist`              | `geometry_msgs::msg::TwistStamped`                     | linear velocity and angular velocity of prediction update |
| `/twist_cov`          | `geometry_msgs::msg::TwistWithCovarianceStamped`       | linear velocity and angular velocity of prediction update |
| `/weighted_particles` | `yabloc_particle_filter::msg::ParticleArray` | particles weighted  by corrector nodes                    |
| `/height`             | `std_msgs::msg::Float32`                               | ground height                                             |

### Output

| Name                          | Type                                                   | Description                           |
|-------------------------------|--------------------------------------------------------|---------------------------------------|
| `/predicted_particles`        | `yabloc_particle_filter::msg::ParticleArray` | particles weighted by predictor nodes |
| `/predicted_particles_marker` | `visualization_msgs::msg::MarkerArray`                 | markers for particle visualization    |
| `/pose`                       | `geometry_msgs::msg::PoseStamped`                      | weighted mean of particles            |
| `/pose_with_covariance`       | `geometry_msgs::msg::PoseWithCovarianceStamped`        | weighted mean of particles            |

### Parameters

| Name                          | Type   | Default    | Description                                                                       |
|-------------------------------|--------|------------|-----------------------------------------------------------------------------------|
| `prediction_rate`             | double | 50         | frequency of forecast updates, in Hz                                              |
| `visualize`                   | bool   | false      | whether particles are also published in visualization_msgs or not                 |
| `num_of_particles`            | int    | 500        | the number of particles                                                           |
| `resampling_interval_seconds` | double | 1.0        | the interval of particle resamping                                                |
| `static_linear_covariance`    | double | 0.01       | to override the covariance of `/twist`. When using `/twist_cov`, it has no effect |
| `static_angular_covariance`   | double | 0.01       | to override the covariance of `/twist`. When using `/twist_cov`, it has no effect |
| `cov_xx_yy`   　　　　　　　　| list   | [2.0,0.25] | the covariance of initial pose                                                    |


## Other nodes
- [camera_corrector](docs/camera_corrector.md)
- [gnss_corrector](docs/gnss_corrector.md)
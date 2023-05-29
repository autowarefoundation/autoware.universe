# Camera Particle Corrector

## Purpose

- A package that weight particles using Camera.

## Inputs / Outputs

### Input

| Name                                                  | Type                                                   | Description                                                 |
|-------------------------------------------------------|--------------------------------------------------------|-------------------------------------------------------------|
| `/predicted_particles`                                | `modularized_particle_filter_msgs::msg::ParticleArray` | predicted particles                                         |
| `/localization/map/ll2_road_marking`                  | `sensor_msgs::msg::PointCloud2`                        | road surface marking converted to line segments             |
| `/localization/imgproc/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2`                        | projected line segments                                     |
| `/pose`                                               | `geometry_msgs::msg::PoseStamped`                      | reference to retrieve the area map around the self location |


### Output

| Name                  | Type                                                   | Description                              |
|-----------------------|--------------------------------------------------------|------------------------------------------|
| `/weighted_particles` | `modularized_particle_filter_msgs::msg::ParticleArray` | weighted particles                       |
| `/cost_map_image`     | `sensor_msgs::msg::Image`                              | cost map created from lanelet2           |
| `/cost_map_range`     | `visualization_msgs::msg::MarkerArray`                 | cost map boundary                        |
| `/match_image`        | `sensor_msgs::msg::Image`                              | projected line segments image            |
| `/scored_cloud`       | `sensor_msgs::msg::PointCloud2`                        | weighted 3d line segments                |
| `/scored_post_cloud`  | `sensor_msgs::msg::PointCloud2`                        | weighted 3d line segments which are iffy |

## Parameters

| Name              | Type  | Default | Description                                                                |
|-------------------|-------|---------|----------------------------------------------------------------------------|
| `max_range`       | float | 80      | width of heararchical cost map                                             |
| `gamma`           | float | 40.0    | gamma value of the intensity gradient of the cost map                      |
| `min_prob`        | float | 0.1     | minimum particle weight the corrector node gives                           |
| `far_weight_gain` | float | 0.001   | `exp(-far_weight_gain_ * squared_distance_from_camera)` is reflected in the weight (If this is large, the nearby landmarks will be more important.)|
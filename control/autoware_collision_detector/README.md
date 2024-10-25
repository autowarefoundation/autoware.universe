# Collision Checker

## Purpose

This module subscribes required data (ego-pose, obstacles, etc), and publishes diagnostics if an object is within a specific distance.

## Inner-workings / Algorithms

### Flow chart

### Algorithms

### Check data

Check that `collision_detector` receives no ground pointcloud, dynamic objects.

### Get distance to nearest object

Calculate distance between ego vehicle and the nearest object.
In this function, it calculates the minimum distance between the polygon of ego vehicle and all points in pointclouds and the polygons of dynamic objects.

## Inputs / Outputs

### Input

| Name                                           | Type                                              | Description                                                        |
| ---------------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------ |
| `/perception/obstacle_segmentation/pointcloud` | `sensor_msgs::msg::PointCloud2`                   | Pointcloud of obstacles which the ego-vehicle should stop or avoid |
| `/perception/object_recognition/objects`       | `autoware_perception_msgs::msg::PredictedObjects` | Dynamic objects                                                    |
| `/tf`                                          | `tf2_msgs::msg::TFMessage`                        | TF                                                                 |
| `/tf_static`                                   | `tf2_msgs::msg::TFMessage`                        | TF static                                                          |

### Output

| Name           | Type                                    | Description |
| -------------- | --------------------------------------- | ----------- |
| `/diagnostics` | `diagnostic_msgs::msg::DiagnosticArray` | Diagnostics |

## Parameters

| Name                 | Type     | Description                                                      | Default value |
| :------------------- | :------- | :--------------------------------------------------------------- | :------------ |
| `use_pointcloud`     | `bool`   | Use pointcloud as obstacle check                                 | `false`       |
| `use_dynamic_object` | `bool`   | Use dynamic object as obstacle check                             | `true`        |
| `collision_distance` | `double` | If objects exist in this distance, publish error diagnostics [m] | 0.1           |

## Assumptions / Known limits

- This module is based on `surround_obstacle_checker`

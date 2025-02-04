# gnss_poser

## Purpose

The `gnss_poser` is a node that subscribes gnss sensing messages and calculates vehicle pose with covariance.

This node subscribes to NavSatFix to publish the pose of **base_link**. The data in NavSatFix represents the antenna's position. Therefore, it performs a coordinate transformation using the tf from `base_link` to the antenna's position. The frame_id of the antenna's position refers to NavSatFix's `header.frame_id`.
(**Note that `header.frame_id` in NavSatFix indicates the antenna's frame_id, not the Earth or reference ellipsoid.** [See also NavSatFix definition.](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))

If the transformation from `base_link` to the antenna cannot be obtained, it outputs the pose of the antenna position without performing coordinate transformation.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                           | Type                                                    | Description                                                                                                                    |
| ------------------------------ | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `/map/map_projector_info`      | `autoware_map_msgs::msg::MapProjectorInfo`              | map projection info                                                                                                            |
| `~/input/fix`                  | `sensor_msgs::msg::NavSatFix`                           | gnss status message                                                                                                            |
| `~/input/autoware_orientation` | `autoware_sensing_msgs::msg::GnssInsOrientationStamped` | orientation [click here for more details](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs) |

### Output

| Name                     | Type                                            | Description                                                    |
| ------------------------ | ----------------------------------------------- | -------------------------------------------------------------- |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`               | vehicle pose calculated from gnss sensing data                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | vehicle pose with covariance calculated from gnss sensing data |
| `~/output/gnss_fixed`    | `tier4_debug_msgs::msg::BoolStamped`            | gnss fix status                                                |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_gnss_poser/schema/gnss_poser.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

# autoware_mission_details_overlay_rviz_plugin

Plugin for displaying 2D overlays over the RViz2 3D scene for mission details (such as remaining distance and time).

Based on the [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)
package, under the 3-Clause BSD license.

## Purpose

This plugin provides a visual and easy-to-understand display of mission details (remaining distance and time)

## Inputs / Outputs

### Input

| Name                                        | Type                                                        | Description                                               |
| ------------------------------------------- | ----------------------------------------------------------- | --------------------------------------------------------- |
| `/planning/mission_remaining_distance_time` | `autoware_planning_msgs::msg::MissionRemainingDistanceTime` | The topic is for mission remaining distance and time Data |

## Parameter

### Core Parameters

#### SignalDisplay

| Name               | Type | Default Value | Description                       |
| ------------------ | ---- | ------------- | --------------------------------- |
| `property_width_`  | int  | 225           | Width of the plotter window [px]  |
| `property_height_` | int  | 100           | Height of the plotter window [px] |
| `property_left_`   | int  | 10            | Left of the plotter window [px]   |
| `property_top_`    | int  | 10            | Top of the plotter window [px]    |

Note that mission details display is aligned with top right corner of the screen.

## Assumptions / Known limits

TBD.

## Usage

Similar to [autoware_overlay_rviz_plugin](../autoware_overlay_rviz_plugin/README.md)

## Credits

### Icons

- https://fonts.google.com/icons?selected=Material+Symbols+Outlined:conversion_path:FILL@1;wght@400;GRAD@200;opsz@20&icon.size=20&icon.color=%23e8eaed&icon.query=path
- https://fonts.google.com/icons?selected=Material+Symbols+Outlined:av_timer:FILL@1;wght@400;GRAD@200;opsz@20&icon.size=20&icon.color=%23e8eaed&icon.query=av+timer

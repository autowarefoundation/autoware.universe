# tier4_debug_rviz_plugin

This package is including jsk code.
Note that jsk_overlay_utils.cpp and jsk_overlay_utils.hpp are BSD license.

## Purpose

This plugin displays the data in `tier4_debug_msgs::msg::Float32MultiArrayStamped`.

## Usage

1. Start rviz and select Add under the Displays panel.
   ![select_add](./images/select_add.png)
2. Select any one of the tier4_planning_rviz_plugin and press OK.
   ![select_planning_plugin](./images/select_planning_plugin.png)
3. Enter the name of the topic where you want to view the path or trajectory.
   ![select_topic_name](./images/select_topic_name.png)

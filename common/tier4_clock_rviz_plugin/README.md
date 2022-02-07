# tier4_clock_rviz_plugin

## Purpose

This plugin allows publishing and controlling the simulated ROS time.

## Inputs / Outputs

### Input

### Output

| Name     | Type                        | Description                |
| -------- | --------------------------- | -------------------------- |
| `/clock` | `rosgraph_msgs::msg::Clock` | the current simulated time |

## HowToUse

1. Start rviz and select panels/Add new panel.
   ![select_panel](./images/select_panels.png)
2. Select tier4_clock_rviz_plugin/SimulatedClock and press OK.
   ![select_state_plugin](./images/select_state_plugin.png)

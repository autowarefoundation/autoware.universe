# mrm_comfortable_stop_operator

## Purpose

MRM comfortable stop operator is a node that generates comfortable stop commands according to the comfortable stop MRM order.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

### Output

| Name                                   | Type                                                  | Description                  |
| -------------------------------------- | ----------------------------------------------------- | ---------------------------- |
| `~/output/velocity_limit`              | `tier4_planning_msgs::msg::VelocityLimit`             | Velocity limit command       |
| `~/output/velocity_limit/clear`        | `tier4_planning_msgs::msg::VelocityLimitClearCommand` | Velocity limit clear command |

## Parameters

### Core Parameters

| Name             | Type   | Default value | Explanation                                       |
| ---------------- | ------ | ------------- | ------------------------------------------------- |
| min_acceleration | double | `-1.0`        | Minimum acceleration for comfortable stop [m/s^2] |
| max_jerk         | double | `0.3`         | Maximum jerk for comfortable stop [m/s^3]         |
| min_jerk         | double | `-0.3`        | Minimum jerk for comfortable stop [m/s^3]         |

## Assumptions / Known limits

TBD.

# Safe Velocity Adjustor

## Purpose

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                 | Type                                     | Description          |
| -------------------- | ---------------------------------------- | -------------------- |
| `~/input/trajectory` | `autoware_auto_planning_msgs/Trajectory` | Reference trajectory |

### Output

| Name                                           | Type                                     | Description                                                |
| ---------------------------------------------- | ---------------------------------------- | ---------------------------------------------------------- |
| `~/output/trajectory`                          | `autoware_auto_planning_msgs/Trajectory` | Modified trajectory                                        |
| `~/debug/trajectory_external_velocity_limited` | `autoware_auto_planning_msgs/Trajectory` | External velocity limited trajectory (for debug)           |
| `~/debug/trajectory_lateral_acc_filtered`      | `autoware_auto_planning_msgs/Trajectory` | Lateral acceleration limit filtered trajectory (for debug) |

## Parameters

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

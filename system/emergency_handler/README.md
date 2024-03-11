# emergency_handler

## Purpose

Emergency Handler is a node to select proper MRM from system failure state contained in HazardStatus.

## Inner-workings / Algorithms

Emergency Handler has MRM operators inside.
Currently, there are two types of MRM:

 - Emergency Stop Operator: publishes `/system/emergency/control_cmd` to make an emergency stop
 - Comfortable Stop Operator: publishes `/planning/scenario_planning/max_velocity_candidates` to make a comfortable stop

These operators have `operate()`, `cancel()` and `onTimer` public functions.

 - `operate()`: executes the MRM operation
 - `cancel()`: cancels the MRM operation
 - `onTimer()`: executes each operator's timer callback

### State Transitions

![fail-safe-state](image/fail-safe-state.drawio.svg)

## Inputs / Outputs

### Input

| Name                                      | Type                                                       | Description                                                                   |
| ----------------------------------------- | ---------------------------------------------------------- | ----------------------------------------------------------------------------- |
| `/system/emergency/hazard_status`         | `autoware_auto_system_msgs::msg::HazardStatusStamped`      | Used to select proper MRM from system failure state contained in HazardStatus |
| `/localization/kinematic_state`           | `nav_msgs::msg::Odometry`                                  | Used to decide whether vehicle is stopped or not                              |
| `/vehicle/status/control_mode`            | `autoware_auto_vehicle_msgs::msg::ControlModeReport`       | Used to check vehicle mode: autonomous or manual                              |
| `/control/command/control_cmd`            | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Used to calculate emergency control commands in emergency_stop_operator       |

### Output

| Name                                                  | Type                                                       | Description                                           |
| ----------------------------------------------------- | ---------------------------------------------------------- | ----------------------------------------------------- |
| `/system/emergency/shift_cmd`                         | `autoware_auto_vehicle_msgs::msg::GearCommand`             | Required to execute proper MRM (send gear cmd)        |
| `/system/emergency/hazard_cmd`                        | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | Required to execute proper MRM (send turn signal cmd) |
| `/api/fail_safe/mrm_state`                            | `autoware_adapi_v1_msgs::msg::MrmState`                    | Inform MRM execution state and selected MRM behavior  |
| `/system/emergency/control_cmd`                       | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Required to operate emergency_stop                    |
| `/planning/scenario_planning/max_velocity_candidates` | `tier4_planning_msgs::msg::VelocityLimit`                  | Required to operate comfortable_stop                  |
| `/planning/scenario_planning/clear_velocity_limit`    | `tier4_planning_msgs::msg::VelocityLimitClearCommand`      | Required to cancel comfortable_stop                   |

## Parameters

{{ json_to_markdown("system/emergency_handler/schema/emergency_handler.schema.json") }}

## Assumptions / Known limits

TBD.

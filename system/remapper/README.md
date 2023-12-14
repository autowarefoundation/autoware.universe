# remapper

## Purpose

Remapper is a node to remap some topics to topics added hostname prefix for multiple ECU configuration.

## Inputs / Outputs

### Input

| Name                                     | Type                                                           |
| ---------------------------------------- | -------------------------------------------------------------- |
| `/system/operation_mode/availability`    | `tier4_system_msgs::msg::OperationModeAvailability`            |
| `/j6/to_can_bus`                         | `can_msgs::msg::Frame`                                         |
| `/control/command/control_cmd`           | `autoware_auto_control_msgs::msg::AckermannControlCommand`     |
| `/tf`                                    | `tf2_msgs::msg::TFMessage`                                     |
| `/api/operation_mode/state`              | `autoware_adapi_v1_msgs::msg::OperationModeState`              |
| `/api/localization/initialization_state` | `autoware_adapi_v1_msgs::msg::LocalizationInitializationState` |
| `/localization/pose_with_covariance`     | `geometry_msgs::msg::PoseWithCovarianceStamped`                |
| `/api/routing/state`                     | `autoware_adapi_v1_msgs::msg::RouteState`                      |
| `/api/routing/route`                     | `autoware_adapi_v1_msgs::msg::Route`                           |
| `/autoware/state`                        | `autoware_auto_system_msgs::msg::AutowareState`                |
| `/vehicle/status/control_mode`           | `autoware_auto_vehicle_msgs::msg::ControlModeReport`           |
| `/api/external/get/emergency`            | `tier4_external_api_msgs::msg::Emergency`                      |
| `/api/fail_safe/mrm_state`               | `autoware_adapi_v1_msgs::msg::MrmState`                        |
| `/diagnostics_graph`                     | `tier4_system_msgs::msg::DiagnosticGraph`                      |
| `/diagnostics_graph/supervisor`          | `tier4_system_msgs::msg::DiagnosticGraph`                      |

### Output

| Name                                                   | Type                                                           |
| ------------------------------------------------------ | -------------------------------------------------------------- |
| `/(main or sub)/system/operation_mode/availability`    | `tier4_system_msgs::msg::OperationModeAvailability`            |
| `/(main or sub)/j6/to_can_bus`                         | `can_msgs::msg::Frame`                                         |
| `/(main or sub)/control/command/control_cmd`           | `autoware_auto_control_msgs::msg::AckermannControlCommand`     |
| `/(main or sub)/tf`                                    | `tf2_msgs::msg::TFMessage`                                     |
| `/(main or sub)/api/operation_mode/state`              | `autoware_adapi_v1_msgs::msg::OperationModeState`              |
| `/(main or sub)/api/localization/initialization_state` | `autoware_adapi_v1_msgs::msg::LocalizationInitializationState` |
| `/(main or sub)/localization/pose_with_covariance`     | `geometry_msgs::msg::PoseWithCovarianceStamped`                |
| `/(main or sub)/api/routing/state`                     | `autoware_adapi_v1_msgs::msg::RouteState`                      |
| `/(main or sub)/api/routing/route`                     | `autoware_adapi_v1_msgs::msg::Route`                           |
| `/(main or sub)/autoware/state`                        | `autoware_auto_system_msgs::msg::AutowareState`                |
| `/(main or sub)/vehicle/status/control_mode`           | `autoware_auto_vehicle_msgs::msg::ControlModeReport`           |
| `/(main or sub)/api/external/get/emergency`            | `tier4_external_api_msgs::msg::Emergency`                      |
| `/(main or sub)/api/fail_safe/mrm_state`               | `autoware_adapi_v1_msgs::msg::MrmState`                        |
| `/(main or sub)/diagnostics_graph`                     | `tier4_system_msgs::msg::DiagnosticGraph`                      |
| `/(main or sub)/diagnostics_graph/supervisor`          | `tier4_system_msgs::msg::DiagnosticGraph`                      |

## Parameters

none

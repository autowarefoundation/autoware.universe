## Purpose

The `redundancy_relay_manager` node subscribes to the election status topics from both the Main ECU and Sub ECU. It manages topic relays between these ECUs, ensuring that when the control path changes, the topic relay from the Main ECU to the Sub ECU stops.

## Inputs / Outputs

### Input

| Name                                | Type                                      | Description                                     |
| ----------------------------------- | ----------------------------------------- | ----------------------------------------------- |
| `~/input/main/election/status`      | `tier4_system_msgs::msg::ElectionStatus`  | Election status topic from the Main ECU.        |
| `~/input/sub/election/status`       | `tier4_system_msgs::msg::ElectionStatus`  | Election status topic from the Sub ECU.         |
| `~/input/operation_mode/state`      | `autoware_adapi_v1_msgs::msg::OperationModeState` | Current operation mode of the system. |

### Output

| Name                                             | Type                                            | Description                                 |
| ------------------------------------------------ | ----------------------------------------------- | ------------------------------------------- |
| `~/output/topic_relay_controller_trajectory/operate` | `tier4_system_msgs::srv::ChangeTopicRelayControl` | Service to control trajectory topic relay.  |
| `~/output/topic_relay_controller_pose_with_covariance/operate` | `tier4_system_msgs::srv::ChangeTopicRelayControl` | Service to control pose topic relay.       |

## Parameters

| Name                  | Type   | Default Value                            | Description                                                                 |
| --------------------- | ------ | ---------------------------------------- | --------------------------------------------------------------------------- |
| `service_timeout_ms`  | `int`  | `1000`                                   | Timeout duration (in milliseconds) for service calls.                       |

## Assumptions / Known limits

- The node assumes the availability of the election status topics (`~/input/main/election/status` and `~/input/sub/election/status`) and the operation mode state (`~/input/operation_mode/state`).
- The node dynamically enables or disables topic relays based on the `path_info` field in the election status messages.
- The system relies on proper remapping and configuration of input and output topics through the launch file.
- Relay behavior is controlled through service calls, which are subject to timeout if the service server does not respond in time.

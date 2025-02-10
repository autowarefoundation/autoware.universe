# autoware_dummy_infrastructure

This is a debug node for infrastructure communication.

## Usage

```sh
ros2 launch autoware_dummy_infrastructure dummy_infrastructure.launch.xml
ros2 run rqt_reconfigure rqt_reconfigure
```

## Inputs / Outputs

### Inputs

| Name                    | Type                                              | Description            |
| ----------------------- | ------------------------------------------------- | ---------------------- |
| `~/input/command_array` | `tier4_v2x_msgs::msg::InfrastructureCommandArray` | Infrastructure command |

### Outputs

| Name                   | Type                                                 | Description                 |
| ---------------------- | ---------------------------------------------------- | --------------------------- |
| `~/output/state_array` | `tier4_v2x_msgs::msg::VirtualTrafficLightStateArray` | Virtual traffic light array |

## Parameters

### Node Parameters

| Name                | Type   | Default Value | Explanation                                       |
| ------------------- | ------ | ------------- | ------------------------------------------------- |
| `update_rate`       | double | `10.0`        | Timer callback period [Hz]                        |
| `use_first_command` | bool   | `true`        | Consider instrument id or not                     |
| `use_command_state` | bool   | `false`       | Consider command state or not                     |
| `instrument_id`     | string | ``            | Used as command id                                |
| `approval`          | bool   | `false`       | set approval filed to ros param                   |
| `is_finalized`      | bool   | `false`       | Stop at stop_line if finalization isn't completed |

## Assumptions / Known limits

TBD.

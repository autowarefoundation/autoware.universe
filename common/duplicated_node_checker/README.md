# dup

## Purpose

This node monitors the ROS2 environments and detect duplication of node names in the envionrment.
The result is publushed as diagnostics.

### Standalone Startup

```bash
ros2 launch duplicated_node_checker duplicated_node_checker.launch.xml
```

## Inner-workings / Algorithms

The types of topic status and corresponding diagnostic status are following.

| Duplication status    | Diagnostic status | Description                |
| --------------------- | ----------------- | -------------------------- |
| `OK`                  | OK                | No duplication is detected |
| `Duplicated Detected` | ERROR             | Duplication is detected    |

## Inputs / Outputs

### Output

| Name           | Type                              | Description         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics outputs |

## Parameters

| Name          | Type   | Default Value | Description                |
| ------------- | ------ | ------------- | -------------------------- |
| `update_rate` | double | 10.0           | Timer callback period [Hz] |

## Assumptions / Known limits

TBD.

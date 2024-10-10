# topic_state_monitor

## Purpose

This node monitors input topic for abnormalities such as timeout and low frequency.
The result of topic status is published as diagnostics.

## Inner-workings / Algorithms

The types of topic status and corresponding diagnostic status are following.

| Topic status  | Diagnostic status | Description                                          |
| ------------- | ----------------- | ---------------------------------------------------- |
| `OK`          | OK                | The topic has no abnormalities                       |
| `NotReceived` | ERROR             | The topic has not been received yet                  |
| `WarnRate`    | WARN              | The frequency of the topic is dropped                |
| `ErrorRate`   | ERROR             | The frequency of the topic is significantly dropped  |
| `Timeout`     | ERROR             | The topic subscription is stopped for a certain time |

## Inputs / Outputs

### Input

| Name     | Type     | Description                       |
| -------- | -------- | --------------------------------- |
| any name | any type | Subscribe target topic to monitor |

### Output

| Name           | Type                              | Description         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics outputs |

## Parameters

{{ json_to_markdown("system/topic_state_monitor/schema/topic_state_monitor.schema.json") }}

## Assumptions / Known limits

TBD.

# System diagnostic graph

## Overview

This package subscribes to a topic of type DiagnosticArray and publishes aggregated diagnostic status.

![overview](./doc/overview.drawio.svg)

## Interface

| Interface Type | Interface Name              | Data Type                               | Description        |
| -------------- | --------------------------- | --------------------------------------- | ------------------ |
| subscription   | `/diagnostics`              | `diagnostic_msgs/msg/DiagnosticArray`   | Input diagnostics. |
| publisher      | `/diagnostics_graph_status` | `diagnostic_msgs/msg/DiagnosticArray`   | Graph status.      |
| publisher      | `/diagnostics_graph_struct` | `tier4_system_msgs/msg/DiagnosticGraph` | Graph structure.   |

## Parameters

| Name               | Type     | Description                                |
| ------------------ | -------- | ------------------------------------------ |
| `config_file`      | `string` | Path of the config file.                   |
| `update_rate`      | `double` | Rate of aggregation and topic publication. |
| `status_qos_depth` | `uint`   | QoS depth of status topic.                 |
| `source_qos_depth` | `uint`   | QoS depth of source topic.                 |

## Config

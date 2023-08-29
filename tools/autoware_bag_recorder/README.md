# autoware_bag_recorder

## Purpose

autoware_bag_recorder is a package for recording necessary topics at bag files. In order to do that, the package have some configurations for recording specifications.

## Inputs / Outputs

### Input

| Name                         | Type                                | Description                                                            |
|------------------------------|-------------------------------------|------------------------------------------------------------------------|
| `/control/current_gate_mode` | `tier4_control_msgs::msg::GateMode` | Required to record bag file in enable_only_auto_mode_recording is true |
| any name                     | any type                            | Subscribe target topic to save in a bag                                |       

### Output

The package does not have output topic.

## Parameters

### Core Parameters

| Name                            | Type         | Default Value | Description                                                   |
|---------------------------------|--------------|---------------|---------------------------------------------------------------|
| `database_storage`              | string       | `"sqlite3"`   | detected objects with score less than threshold are ignored   |
| `maximum_record_time`           | int          | 7200          | [s] the world frame id to fuse multi-frame pointcloud         |

## Assumptions / Known limits


## References/External links

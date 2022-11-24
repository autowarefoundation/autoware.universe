# Drivablea Area Expander

## Purpose

This node allow expanding the drivable area contained in a `Path` message.

## Inner-workings / Algorithms

## Inputs / Outputs

### Inputs

| Name                      | Type                                             | Description                                  |
| ------------------------- | ------------------------------------------------ | -------------------------------------------- |
| `~/input/path`            | `autoware_auto_planning_msgs/Path`               | Reference path                               |
| `~/input/dynamic_objects` | `autoware_auto_perception_msgs/PredictedObjects` | Dynamic objects                              |
| `~/input/map`             | `autoware_auto_mapping_msgs/HADMapBin`           | Vector map used to retrieve static obstacles |

### Outputs

| Name                            | Type                               | Description                                        |
| ------------------------------- | ---------------------------------- | -------------------------------------------------- |
| `~/output/path`                 | `autoware_auto_planning_msgs/Path` | Path with adjusted velocities                      |
| `~/output/debug_markers`        | `visualization_msgs/MarkerArray`   | Debug markers (envelopes, obstacle polygons)       |
| `~/output/runtime_microseconds` | `std_msgs/Int64`                   | Time taken to calculate the path (in microseconds) |

## Parameters

| Name      | Type   | Description                                                                                                   |
| --------- | ------ | ------------------------------------------------------------------------------------------------------------- |
| `min_ttc` | double | [s] required minimum time with no collision at each point of the path assuming constant heading and velocity. |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

# dummy_perception_publisher

## Purpose

This node publishes the result of the dummy detection with the type of perception.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name           | Type                                      | Description             |
| -------------- | ----------------------------------------- | ----------------------- |
| `/tf`          | `tf2_msgs/TFMessage`                      | TF (self-pose)          |
| `input/object` | `tier4_simulation_msgs::msg::DummyObject` | dummy detection objects |

### Output

| Name                                | Type                                                     | Description             |
| ----------------------------------- | -------------------------------------------------------- | ----------------------- |
| `output/dynamic_object`             | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | dummy detection objects |
| `output/points_raw`                 | `sensor_msgs::msg::PointCloud2`                          | point cloud of objects  |
| `output/debug/ground_truth_objects` | `autoware_perception_msgs::msg::TrackedObjects`          | ground truth objects    |

## Parameters

{{json_to_markdown("simulator/dummy_perception_publisher/schema/dummy_perception_publisher.schema.json")}}

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

TBD.

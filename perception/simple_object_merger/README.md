# simple_object_merger

This package can merge multiple topics of [autoware_auto_perception_msgs/msg/DetectedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/DetectedObject.idl) without data association algorithm.

## Algorithm

### Background

This package can merge multiple DetectedObjects without matching processing.
[Object_merger](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/object_merger) solve data association algorithm like Hungarian algorithm for matching problem, but it needs computational cost.
In addition, for now, [object_merger](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/object_merger) can handle only 2 DetectedObjects topics and cannot handle more than 2 topics in one node.
To merge 6 DetectedObjects topics, 6 [object_merger](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/object_merger) nodes need to stand.

So this package aim to merge DetectedObjects simply.
This package do not use data association algorithm to reduce the computational cost, and it can handle more than 2 topics in one node to prevent launching a large number of nodes.

### Limitation

Because this package does not have matching processing, so it does not use without post-processing.

Use case is as below.

- Multiple radar detection

This package can be used for multiple radar detection.
Since [clustering processing](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/radar_object_clustering) will be included later process in radar faraway detection, this package can be used.

## Input

| Name | Type                                                               | Description                                            |
| ---- | ------------------------------------------------------------------ | ------------------------------------------------------ |
|      | std::vector<autoware_auto_perception_msgs/msg/DetectedObjects.msg> | 3D detected objects. Topic names are set by parameters |

## Output

| Name               | Type                                                  | Description    |
| ------------------ | ----------------------------------------------------- | -------------- |
| `~/output/objects` | autoware_auto_perception_msgs/msg/DetectedObjects.msg | Merged objects |

## Parameters

| Name             | Type         | Description                         | Default value |
| :--------------- | :----------- | :---------------------------------- | :------------ |
| `update_rate_hz` | double       | Update rate. [hz]                   | 20.0          |
| `new_frame_id`   | string       | The header frame_id of output topic | "base_link"   |
| `input_topics`   | List[string] | Input topics name                   | "[]"          |

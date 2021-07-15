# Naming Conventions

## Package Names

Although Autoware does not have its own explicit naming convention, it does adhere to the guidance given in [REP-144](https://www.ros.org/reps/rep-0144.html). Thus an Autoware package name must:

> - only consist of lowercase alphanumerics and \_ separators, and start with an alphabetic character
> - not use multiple \_ separators consecutively
> - be at least two characters long

## Topic Names

### Default topic names

In Autoware, all topics should be named according to the guidelines in the [ROS wiki](http://wiki.ros.org/Names).
Additionally, it is strongly recommended that the default topic names specified in source code should follow these conventions:

- All topics must be set under private namespaces. Any global topics must have a documented explanation.
- All topics must be specified under one of the following namespaces within the node's private namespace. Doing so allows users to easily understand which topics are inputs and which are outputs when they look at remapping in launch files for example.
  - `input`: subscribed topics
  - `output`: published topics
  - `debug`: published topics that are meant for debugging (e.g. for visualization)

Consider, for example, a node that subscribes to pointcloud data, applies a voxel grid filter and then publishes the filtered data. In this case, the topics should be named as follows:

- ~input/points_original
- ~output/points_filtered

### Remapped Topics

The default topics of each node can be remapped to other topic names using a launch file.
For encapsulation purposes and ease of understanding, remapped topics should be published under the namespaces of the appropriate modules as per Autoware's layered architecture. Doing so allows both developers and users to see at a glance where the topic is used in the architecture.

Some key topics are listed below:

```txt
/control/vehicle_cmd
/perception/object_recognition/detection/objects
/perception/object_recognition/objects
/perception/object_recognition/tracking/objects
/perception/traffic_light_recognition/traffic_light_states
/planning/mission_planning/route
/planning/scenario_planning/lane_driving/behavior_planning/path
/planning/scenario_planning/lane_driving/trajectory
/planning/scenario_planning/parking/trajectory
/planning/scenario_planning/scenario
/planning/scenario_planning/scenario_selector/trajectory
/planning/scenario_planning/trajectory
```

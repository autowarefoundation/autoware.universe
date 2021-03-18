# Naming Conventions

## Package Names
Autoware does not set any particular naming convention except that it follows [REP-144](https://www.ros.org/reps/rep-0144.html).
Therefore, package name must:
* only consist of lowercase alphanumerics and _ separators and start with an alphabetic character
* not use multiple _ separators consecutively
* be at least two characters long

See [REP-144](https://www.ros.org/reps/rep-0144.html) for the details.

## Topic Names

### Default topic names
In Autoware, all topic name should be following this [wiki](http://wiki.ros.org/Names).
Also, it is strongly recommended that the default topic names specified in source code must follow these conventions:
* All topics must be set under private namespace. Any global topics must have an explained in documentation.
* All topics must be specified under following namespace in the node's private namespace. This allows users to easily understand which topics are inputs and outputs when they look at remapping in launch files for example.
  * `input`: subscribed topics
  * `output`: published topics 
  * `debug`: published topics that are meant for debugging (e.g. for visualization)

For example, if there is a node that subscribes pointcloud and filter it will voxel grid filter, the topics should be:
* ~input/points_original
* ~output/points_filtered

### Remapped Topics
The default topics of each node may be remapped to other topic names in launch file.
In general, the topics should be published under namespaces of belonging modules in layered architecture for encapsulation.
This allows the developers and users to easily understand where in the architecture topic is used.

Some other key topics are listed below:
```
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
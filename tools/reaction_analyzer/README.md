# Reaction Analyzer

## Description

The main purpose of the reaction analyzer package is to measure the reaction times of various nodes within a ROS-based
autonomous driving simulation environment by subscribing to pre-determined topics. This tool is particularly useful for
evaluating the performance of perception, planning, and control pipelines in response to dynamic changes in the
environment, such as sudden obstacles. To be able to measure both control outputs and perception outputs, it was
necessary to divide the node into two running_mode: `planning_control` and `perception_planning`.

![ReactionAnalyzerDesign.png](media%2FReactionAnalyzerDesign.png)

### Planning Control Mode

In this mode, the reaction analyzer creates a dummy publisher for the PredictedObjects and PointCloud2 topics. In the
beginning of the test, it publishes the initial position of the ego vehicle and the goal position to set the test
environment. Then, it spawns a sudden obstacle in front of the ego vehicle. After the obstacle is spawned, it starts to
search reacted messages of the planning and control nodes in the pre-determined topics. When all the topics are reacted,
it calculates the reaction time of the nodes and statistics by comparing `reacted_times` of each of the nodes
with `spawn_cmd_time`, and it creates a csv file to store the results.

### Perception Planning Mode

In this mode, the reaction analyzer reads the rosbag files which are recorded from AWSIM, and it creates a topic
publisher for each topic inside the rosbag to replay the rosbag. It reads two rosbag files: `path_bag_without_object`
and `path_bag_with_object`. Firstly, it replays the `path_bag_without_object` to set the initial position of the ego
vehicle and the goal position. After `spawn_time_after_init` seconds , it replays the `path_bag_with_object` to spawn a
sudden obstacle in front of the ego vehicle. After the obstacle is spawned, it starts to search the reacted messages of
the perception and planning nodes in the pre-determined topics. When all the topics are reacted, it calculates the
reaction time of the nodes and statistics by comparing `reacted_times` of each of the nodes with `spawn_cmd_time`, and
it creates a csv file to store the results.

#### Point Cloud Publisher Type

To get better analyze for Perception & Sensing pipeline, the reaction analyzer can publish the point cloud messages in 3
different ways: `async_header_sync_publish`, `sync_header_sync_publish` or `async_publish`. (`T` is the period of the
lidar's output)

![PointcloudPublisherType.png](media%2FPointcloudPublisherType.png)

- `async_header_sync_publish`: It publishes the point cloud messages synchronously with asynchronous header times. It
  means that each of the lidar's output will be published at the same time, but the headers of the point cloud messages
  includes different timestamps because of the phase difference.
- `sync_header_sync_publish`: It publishes the point cloud messages synchronously with synchronous header times. It
  means that each of the lidar's output will be published at the same time, and the headers of the point cloud messages
  includes the same timestamps.
- `async_publish`: It publishes the point cloud messages asynchronously. It means that each of the lidar's output will
  be published at different times.

## Usage

The common parameters you need to define for both running modes are `output_file_path`, `test_iteration`,
and `reaction_chain` list. `output_file_path` is the output file path is the path where the results and statistics
will be stored. `test_iteration` defines how many tests will be performed. The `reaction_chain` list is the list of the
pre-defined topics you want to measure their reaction times.

### Prepared Test Environment

- Download the demonstration test map from the
  link [here](https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip). After
  downloading,
  extract the zip file and use its path as `[MAP_PATH]` in the following commands.

#### Planning Control Mode

- You need to define only Planning and Control nodes in the `reaction_chain` list. With the default parameters,
  you can start to test with the following command:

```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=planning_control vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit map_path:=[MAP_PATH]
```

After the command, the `simple_planning_simulator` and the `reaction_analyzer` will be launched. It will automatically
start to test. After the test is completed, the results will be stored in the `output_file_path` you defined.

#### Perception Planning Mode

- Download the rosbag files from the Google Drive
  link [here](https://drive.google.com/file/d/1_P-3oy_M6eJ7fk8h5CP8V6s6da6pPrBu/view?usp=sharing).
- Extract the zip file and set the path of the `.db3` files to parameters `path_bag_without_object`
  and `path_bag_with_object`.
- Because custom sensor setup, you need to check out the following branches before launch the
  reaction analyzer: For the `autoware_individual_params` repository, check out the
  branch [here](https://github.com/brkay54/autoware_individual_params/tree/bk/reaction-analyzer-config).
- For the `awsim_sensor_kit_launch` repository, check out the
  branch [here](https://github.com/brkay54/awsim_sensor_kit_launch/tree/bk/reaction-analyzer-config).
- After you check outed the branches, you can start to test with the following command:

```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=perception_planning vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=[MAP_PATH]
```

After the command, the `e2e_simulator` and the `reaction_analyzer` will be launched. It will automatically start
to test. After the test is completed, the results will be stored in the `output_file_path` you defined.

### Custom Test Environment

**If you want to run the reaction analyzer with your custom test environment, you need to redefine some of the
parameters.
The parameters you need to redefine are `initialization_pose`, `entity_params`, `goal_pose`, and `topic_publisher` (
for `perception_planning` mode) parameters.**

- To set `initialization_pose`, `entity_params`, `goal_pose`:
- Upload your `.osm` map file into the [scenario editor](https://scenario.ci.tier4.jp/scenario_editor/) to define the
  position of the position parameters.
- Add EGO vehicle from edit/add entity/Ego to map.
- Set destination to EGO vehicle and add another dummy object in same way. The dummy object represents the object spawn
  suddenly in the reaction analyzer test.

**After you set up the positions in the map, we should get the positions of these entities in the map frame. To achieve
this:**

- Convert the positions to map frame by changing Map/Coordinate to World and Map/Orientation to Euler in Scenario
  Editor.

- After these steps, you can see the positions in map frame and euler angles. You can change
  the `initialization_pose`, `entity_params`, `goal_pose` parameters with the values you get from the website.

**For the `topic_publisher` parameters, you need to record the rosbags from the AWSIM. After opened your AWSIM
environment, you should record two different rosbags. However, the environment should be static and the position of the
vehicle should be same.**

- Record a rosbag in empty environment (without an obstacle in front of the vehicle).
- After that, record another rosbag in the same environment except add an object in front of the vehicle.

**After you record the rosbags, you can set the `path_bag_without_object` and `path_bag_with_object` parameters with the
paths of the recorded rosbags.**

## Parameters

| Name                                                                         | Type   | Description                                                                                                                                   |
| ---------------------------------------------------------------------------- | ------ | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `timer_period`                                                               | double | [s] Period for the main processing timer.                                                                                                     |
| `test_iteration`                                                             | int    | Number of iterations for the test.                                                                                                            |
| `output_file_path`                                                           | string | Directory path where test results and statistics will be stored.                                                                              |
| `object_search_radius_offset`                                                | double | [m] Additional radius added to the search area when looking for objects.                                                                      |
| `spawn_time_after_init`                                                      | double | [s] Time delay after initialization before spawning objects. Only valid `perception_planning` mode.                                           |
| `spawn_distance_threshold`                                                   | double | [m] Distance threshold for spawning objects. Only valid `planning_control` mode.                                                              |
| `spawned_pointcloud_sampling_distance`                                       | double | [m] Sampling distance for point clouds of spawned objects. Only valid `planning_control` mode.                                                |
| `dummy_perception_publisher_period`                                          | double | [s] Publishing period for the dummy perception data. Only valid `planning_control` mode.                                                      |
| `poses.initialization_pose`                                                  | struct | Initial pose of the vehicle, containing `x`, `y`, `z`, `roll`, `pitch`, and `yaw` fields. Only valid `planning_control` mode.                 |
| `poses.entity_params`                                                        | struct | Parameters for entities (e.g., obstacles), containing `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `x_dimension`, `y_dimension`, and `z_dimension`. |
| `poses.goal_pose`                                                            | struct | Goal pose of the vehicle, containing `x`, `y`, `z`, `roll`, `pitch`, and `yaw` fields.                                                        |
| `topic_publisher.path_bag_without_object`                                    | string | Path to the ROS bag file without objects. Only valid `perception_planning` mode.                                                              |
| `topic_publisher.path_bag_with_object`                                       | string | Path to the ROS bag file with objects. Only valid `perception_planning` mode.                                                                 |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_type`             | string | Defines how the PointCloud2 messages are going to be published. Modes explained above.                                                        |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_period`           | double | [s] Publishing period of the PointCloud2 messages.                                                                                            |
| `topic_publisher.pointcloud_publisher.publish_only_pointcloud_with_object`   | bool   | Default false. Publish only the point cloud messages with the object.                                                                         |
| `reaction_params.first_brake_params.debug_control_commands`                  | bool   | Debug publish flag.                                                                                                                           |
| `reaction_params.first_brake_params.control_cmd_buffer_time_interval`        | double | [s] Time interval for buffering control commands.                                                                                             |
| `reaction_params.first_brake_params.min_number_descending_order_control_cmd` | int    | Minimum number of control commands in descending order for triggering brake.                                                                  |
| `reaction_params.first_brake_params.min_jerk_for_brake_cmd`                  | double | [m/s³] Minimum jerk value for issuing a brake command.                                                                                        |
| `reaction_params.search_zero_vel_params.max_looking_distance`                | double | [m] Maximum looking distance for zero velocity on trajectory                                                                                  |
| `reaction_params.search_entity_params.search_radius`                         | double | [m] Searching radius for spawned entity. Distance between ego pose and entity pose.                                                           |
| `reaction_chain`                                                             | struct | List of the nodes with their topics and topic's message types.                                                                                |

## Limitations

- Reaction analyzer has some limitation like `PublisherMessageType`, `SubscriberMessageType` and `ReactionType`. It is
  currently supporting following list:

- **Publisher Message Types:**

  - `sensor_msgs/msg/PointCloud2`
  - `sensor_msgs/msg/CameraInfo`
  - `sensor_msgs/msg/Image`
  - `geometry_msgs/msg/PoseWithCovarianceStamped`
  - `sensor_msgs/msg/Imu`
  - `autoware_auto_vehicle_msgs/msg/ControlModeReport`
  - `autoware_auto_vehicle_msgs/msg/GearReport`
  - `autoware_auto_vehicle_msgs/msg/HazardLightsReport`
  - `autoware_auto_vehicle_msgs/msg/SteeringReport`
  - `autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport`
  - `autoware_auto_vehicle_msgs/msg/VelocityReport`

- **Subscriber Message Types:**

  - `sensor_msgs/msg/PointCloud2`
  - `autoware_auto_perception_msgs/msg/DetectedObjects`
  - `autoware_auto_perception_msgs/msg/TrackedObjects`
  - `autoware_auto_msgs/msg/PredictedObject`
  - `autoware_auto_planning_msgs/msg/Trajectory`
  - `autoware_auto_control_msgs/msg/AckermannControlCommand`

- **Reaction Types:**
  - `FIRST_BRAKE`
  - `SEARCH_ZERO_VEL`
  - `SEARCH_ENTITY`

## Future improvements

- The reaction analyzer can be improved by adding more reaction types. Currently, it is supporting only `FIRST_BRAKE`,
  `SEARCH_ZERO_VEL`, and `SEARCH_ENTITY` reaction types. It can be extended by adding more reaction types for each of
  the
  message types.

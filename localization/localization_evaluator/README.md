# localization_evaluator

## Purpose

`localization_evaluator` is the package to evaluate localization output. The Error is published as `PoseStamped` message and can be visualized using plotjuggler.

## Inputs / Outputs

### Input

| Name                                          | Type                              | Description       |
| --------------------------------------------- | --------------------------------- | ----------------- |
| `/localization/pose_twist_fusion_filter/pose` | `geometry_msgs::msg::PoseStamped` | pose from vehicle |
| `/ground_truth`                               | `geometry_msgs::msg::PoseStamped` | ground truth pose |

### Output

| Name                   | Type                              | Description         |
| ---------------------- | --------------------------------- | ------------------- |
| `/relative_pose_error` | `geometry_msgs::msg::PoseStamped` | relative pose error |

## Parameters

| Parameter    | Type   | Description                                                     |
| ------------ | ------ | --------------------------------------------------------------- |
| `time_delta` | Double | timestamp difference for vehicle pose and ground truth matching |

## Assumptions / Known limits

This module uses RPE(Relative Pose Error) as an evaluation metric. To put simply, a subset of trajectory, a pair of states in this case , is used to compute a relative error. This corresponds to the drift of defined trajectory. One know issue is that we cannot perfectly align a ground truth pose with an output of localization due to unknown processing time and time synchronization of different nodes. Therefore, we interpolate poses from previous and post outputs from localization.

## Instruction To Create Test Data from KITTI Dataset

### 1. Download KITTI Raw data
[KITTI Raw](http://www.cvlibs.net/datasets/kitti/raw_data.php)
`synced+rectified` version is recommended. Set up the directory according to KITTI's instructions.

Tools could be found in the following repo:
[kitti_to_ros2bag](https://github.com/angry-crab/kitti_to_ros2bag)

### 2. Create a pcd map
 - clone the repo above
 - in `create_map.cc`, change `pose_paths` and `pcd_paths` to where your data path, and path of saving output map. 
 - `mkdir build && cd build`, `cmake ..`, `make`
 - run `pcd_mapping` and wait for the map to be generated. 

### 3. Create a ros2 bag
 - in `create_bag.py`, change `base_dir`, `date`, and `drive` to the data you want to use.
 - run `create_bag.py`
 - wait for the bag to be generated.

(A ready-to-go dataset could be found here [sample map and bag](https://drive.google.com/file/d/1hTePvUZ1_tTOefex4UgZqjdibGARjDtr/view?usp=sharing), I'm not sure if it is correctly shared, please let me know if it could not be accessed. )

### 4. Bring up `Universe`
- Follow the instructions below, up to `vcs import src < autoware.repos` (DO NOT BUILD AT THIS TIME) [Autoware Doc](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)
- Terminal 1
`cd src/launcher/autoware_launch`
`git checkout kitti_test`
`cd ../../..`
`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
`source install/setup.bash`
`ros2 launch autoware_launch logging_simulator.launch.xml map_path:=PATH_OF_YOUR_MAP vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit vehicle:=false system:=false map:=true sensing:=true localization:=true perception:=false planning:=false control:=false`
- New Terminal 2
`source ~/autoware/install/setup.bash`
`ros2 launch autoware_launch kitti_static.launch.py`
- New Terminal 3
`source ~/autoware/install/setup.bash`
`ros2 bag play PATH_TO_ROS2BAG/ros2_bag.db3 --qos-profile-overrides-path qos.yaml -d 3`
- New Terminal 4
`source ~/autoware/install/setup.bash`
`ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused`
- Go to Rviz of Terminal 1, set an intial pose, when Terminal 1 output `Alignment called`, go to Terminal 4 and rerun the last command. Then you are good to go. 
 Check the link below for a video demonstration. [Sample Video](https://github.com/orgs/autowarefoundation/discussions/334)

Reference:
[A_benchmark_for_the_evaluation_of_RGB-D_SLAM_systems](https://www.researchgate.net/publication/261353760_A_benchmark_for_the_evaluation_of_RGB-D_SLAM_systems)

Original Issue Link:
[Evaluate performance of localization pipeline](https://github.com/autowarefoundation/autoware.universe/issues/602)
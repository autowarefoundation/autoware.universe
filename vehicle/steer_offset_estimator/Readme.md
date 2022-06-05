# steer_offset_estimator

The role of this node is to automatically calibrate `steer_offset` used in the `vehicle_interface` node.

The base steer offset value is 0 by default, which is standard, is updated iteratively with the loaded driving data.

## How to calibrate

### Launch Calibrator

After launching Autoware, run the `steer_offset_estimator` by the following command and then perform autonomous driving. Note: You can collect data with manual driving if it is possible to use the same vehicle interface as during autonomous driving (e.g. using a joystick).

```sh
ros2 launch steer_offset_estimator steer_offset_estimator.launch.xml
```

Or if you want to use rosbag files, run the following commands.

```sh
ros2 param set /use_sim_time true
ros2 bag play <rosbag_file> --clock
```

### Diagnostics

The `steer_offset_estimator` publishes diagnostics message depending on the calibration status.
Diagnostic type `WARN` indicates that the current steer_offset is estimated to be inaccurate. In this situation, it is strongly recommended to perform a re-calibration of the steer_offset.

| Status                  | Diagnostics Type | Diagnostics message                     |
| ----------------------- | ---------------- | --------------------------------------- | --- |
| No calibration required | `OK`             | "Preparation"                           |     |
| Calibration Required    | `WARN`           | "Steer offset is larger than tolerance" |

This diagnostics status can be also checked on the following ROS topic.

```sh
ros2 topic echo /vehicle/status/steering_offset
```

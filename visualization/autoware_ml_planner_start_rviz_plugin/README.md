# ML planner start RViz panel

RViz panel for the ML planner standstill kick.

Service:

```bash
ros2 service call /planning/trajectory_generator/neural_network_based_planner/ml_planner_node/service/start std_srvs/srv/SetBool "{data: true}"
```

`data: true` overwrites every ego-velocity history entry with 1 m/s so the model plans as if the vehicle is already moving. `data: false` turns that override off.

## Behavior

1. **Start ML planner** calls the service once with `data: true`.
2. The button switches to **WORKING** only after the service returns `success: true`.
3. While WORKING, the panel watches `/localization/kinematic_state`. When planar speed (`hypot(vx, vy)`) exceeds **0.2 m/s**, it calls the service once with `data: false`.
4. **Force stop** is enabled only in WORKING. It calls `data: false` immediately, including when the vehicle has not moved.
5. **DONE** is shown only after that stop call returns `success: true`. A failed stop leaves the panel in WORKING so Force stop or the speed trigger can retry. Start again is available after DONE.

## Build

From the workspace root, with the Autoware overlay sourced:

```bash
colcon build --symlink-install --packages-select autoware_ml_planner_start_rviz_plugin --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

In RViz: Panels → Add New Panel → `autoware_ml_planner_start_rviz_plugin/MLPlannerStartPanel`.

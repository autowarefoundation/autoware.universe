# CARLA Autoware

# ROS2/Autoware.universe bridge for CARLA simulator

Thanks to <https://github.com/gezp> for ROS2 Humble support for CARLA Communication.
This ros package enables communication between Autoware and CARLA for autonomous driving simulation.

# Environment

| ubuntu |  ros   | carla  | autoware |
| :----: | :----: | :----: | :------: |
| 22.04  | humble | 0.9.15 | 2024.04  |

# Setup

## install

- [CARLA Installation](https://carla.readthedocs.io/en/latest/start_quickstart/)
- [Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)
- [Python Package for CARLA 0.9.15 Ros2 Humble communication](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04)

  - Install the wheel using pip.
  - OR add the egg file to the `PYTHONPATH`.

1. Download maps (y-axis inverted version) to arbitaly location
2. Change names and create the map folder (example: Town01) inside `autoware_map`. (`point_cloud/Town01.pcd` -> `autoware_map/Town01/pointcloud_map.pcd`, `vector_maps/lanelet2/Town01.osm`-> `autoware_map/Town01/lanelet2_map.osm`)
3. Create `map_projector_info.yaml` and add `projector_type: local` on the first line.

## build

```bash
cd colcon_ws
colcon build --symlink-install
```

# Run

1. Run carla, change map, spawn object if you need

```bash
cd CARLA
./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
```

2. Run ros nodes

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/Town01 vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit simulator_type:=carla
```

3. Set initial pose (Init by GNSS)
4. Set goal position
5. Wait for planning
6. Engage

# Tips

- If you want to edit the sensors configuration used in CARLA, edit `objects.json` located in `carla_autoware/config`. Make sure the sensor configuration is the same as the `sensor_kit` description used in Autoware.
- Misalignment might occurs during initialization, pressing `init by gnss` button should fix it.

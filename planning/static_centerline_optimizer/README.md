# Static Centerline Optimizer

## Purpose

This package statically calcualtes the centerline satisfying footprints inside the drivable area.

On narrow-road driving, the default centerline, which is the middle line between lanelet's right and left bounds, often causes footprints outside the drivable area.
With the static centerline optimization, we have the following advantages.

- We can see the optimized centerline shape in advance.
- We do not have to calculate a heavy and sometimes unstable path optimization since the footprints are already inside the drivable area.

## Usecases

There are two interfaces to communicate with the centerline optimizer.

### Vector Map Builder Interface

Run the autoware server with the following command by designating `vehicle_model` as lexus for example.

```sh
ros2 run static_centerline_optimizer run_planning_server.sh vehicle_model:=lexus
```

Run http server with the following command.
The port number is 5000 by default.

```sh
ros2 run static_centerline_optimizer app.py
```

### Command Line Interface

The optimized centerline is generated from the command line interface.

```sh
# ros2 run static_centerline_optimizer optimize_path.sh <osm-map-path> <start-lanelet-id> <end-lanelet-id> <vehicle-model>
$ ros2 launch static_centerline_optimizer static_centerline_optimizer.launch.xml lanelet2_input_file_name:="$HOME"/AutonomousDrivingScenarios/map/kashiwanoha/lanelet2_map.osm start_lanelet_id:=125 end_lanelet_id:=132 vehicle_model:=lexus run_backgrond:=false
```

image

The yellow footprints are the original ones in the map file.
The red footprints are the optimized ones.
You can see that the red footprints are inside the lane although the yellow ones are outside.

The output map with the optimized centerline locates `/tmp/lanelet2_map.osm`
If you want to change the output map path, you can remap the path

```sh
ros2 run static_centerline_optimizer optimize_centerline.sh <map-path> <start-lanelet-id> <end-lanelet-id> --ros-args --remap output:=<output-map-path>
```

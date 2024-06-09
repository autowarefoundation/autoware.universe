#!/bin/bash

ros2 launch autoware_static_waypoints_generator static_waypoints_generator.launch.xml waypoints_source:="optimization_trajectory_base" run_background:=false lanelet2_input_file_path:="$HOME/autoware_map/sample_map/lanelet2_map.osm" start_lanelet_id:=125 end_lanelet_id:=132 vehicle_model:=lexus

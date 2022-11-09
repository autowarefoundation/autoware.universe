#!/bin/bash

if [ $# != 4 ]; then
    echo "Invalid arguments"
    echo "Usage: ros2 run static_centerline_optimizer optimize_path.sh <osm-map-path> <start-lanelet-id> <end-lanelet-id> <vehicle-model>"
fi

ros2 launch static_centerline_optimizer static_centerline_optimizer.launch.xml run_backgrond:=false lanelet2_input_file_name:="$1" start_lanelet_id:="$2" end_lanelet_id:="$3" vehicle_model:="$4"

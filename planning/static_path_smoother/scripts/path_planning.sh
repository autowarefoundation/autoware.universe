#!/bin/bash

ros2 launch static_path_smoother path_planning.launch.xml lanelet2_file_name:=/home/takayukimurooka/AutonomousDrivingScenarios/map/kashiwanoha/lanelet2_map.osm start_lanelet_id:=125 end_lanelet_id:=132 vehicle_model:=ymc_golfcart_m0

# ros2 launch static_path_smoother path_planning.launch.xml lanelet2_file_name:=/tmp/popo/lanelet2_map.osm start_lanelet_id:=125 end_lanelet_id:=132 vehicle_model:=ymc_golfcart_m0

# ros2 launch static_path_smoother path_planning.launch.xml lanelet2_file_name:=/home/takayukimurooka/AutonomousDrivingScenarios/map/x1/narrow/centerline/remake_ryuyo_inside_left_Lcurve_2.0x2.0_oval_using_CenterLine0.6m_r3/lanelet2_map.osm start_lanelet_id:=125 end_lanelet_id:=132 vehicle_model:=ymc_golfcart_m0

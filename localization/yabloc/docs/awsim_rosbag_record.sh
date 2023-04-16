#!/usr/bin/bash
ros2 bag record --use-sim-time\
    /clock\
    /tf_static\
    /map/vector_map\
    /map/vector_map_marker\
    /vehicle/status/velocity_status\
    /sensing/imu/tamagawa/imu_raw\
    /sensing/gnss/pose\
    /sensing/gnss/pose_with_covariance\
    /sensing/camera/traffic_light/camera_info\
    /sensing/camera/traffic_light/image_raw/compressed\
    /initialpose

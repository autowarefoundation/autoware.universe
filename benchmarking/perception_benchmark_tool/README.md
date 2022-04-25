This package still under-development

1- Download sample Waymo segment file from: https://waymo.com/open/download/

2- Install the Waymo Open Dataset tool 
```bash
pip3 install waymo-open-dataset-tf-2-4-0 --user
```

3- For running Autoware.Universe perception stack with the Waymo Dataset 
```bash
ros2 run perception_benchmark_tool perception_benchmark_node --ros-args --params-file benchmark.param.yaml
```

Lidar point clouds and camera images are encoded in the .tfrecord file. It may take about ~60-90 seconds to decode 
the data back.  

4- Running perception pipeline:
```bash
ros2 launch benchmarking_launch waymo.launch.xml
```

5- Running RVIZ2
```bash
ros2 run rviz2 rviz2 -d ${AUTOWARE_PATH}/src/universe/autoware/benchmarking/benchmarking_launch/rviz/waymo.rviz
```


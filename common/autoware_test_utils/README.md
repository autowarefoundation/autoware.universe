# Test Utils

## Background

Several Autoware's components and modules have already adopted unit testing, so a common library to ease the process of writing unit tests is necessary.

## Purpose

The objective of the `test_utils` is to develop a unit testing library for the Autoware components. This library will include

- commonly used functions
- input/mock data parser
- maps for testing
- common routes and mock data for testing.

## Available Maps

The following maps are available [here](https://github.com/autowarefoundation/autoware.universe/tree/main/common/autoware_test_utils/test_map)

### Common

The common map contains multiple types of usable inputs, including shoulder lanes, intersections, and some regulatory elements. The common map is named `lanelet2_map.osm` in the folder.

![common](./images/common.png)

### 2 km Straight

The 2 km straight lanelet map consists of two lanes that run in the same direction. The map is named `2km_test.osm`.

![two_km](./images/2km-test.png)

The following illustrates the design of the map.

![straight_diagram](./images/2km-test.svg)

### road_shoulders

The road_shoulders lanelet map consist of a variety of pick-up/drop-off site maps with road_shoulder tags including:

- pick-up/drop-off sites on the side of street lanes
- pick-up/drop-off sites on the side of curved lanes
- pick-up/drop-off sites inside a private area

![road_shoulder_test](./images/road_shoulder_test_map.png)

You can easily launch planning_simulator by

```bash
ros2 launch autoware_test_utils psim_road_shoulder.launch.xml vehicle_model:=<> sensor_model:=<> use_sim_time:=true
```

### intersection

The intersections lanelet map consist of a variety of intersections including:

- 4-way crossing with traffic light
- 4-way crossing without traffic light
- T-shape crossing without traffic light
- intersection with a loop
- complicated intersection

![intersection_test](./images/intersection_test_map.png)

You can easily launch planning_simulator by

```bash
ros2 launch autoware_test_utils psim_intersection.launch.xml vehicle_model:=<> sensor_model:=<> use_sim_time:=true
```

## Example use cases

### Autoware Planning Test Manager

The goal of the [Autoware Planning Test Manager](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_planning_test_manager/) is to test planning module nodes. The `PlanningInterfaceTestManager` class ([source code](https://github.com/autowarefoundation/autoware.universe/blob/main/planning/autoware_planning_test_manager/src/autoware_planning_test_manager.cpp)) creates wrapper functions based on the `test_utils` functions.

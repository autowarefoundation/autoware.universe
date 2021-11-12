Costmap generator {#costmap-generator}
========

This is the design document for the `costmap_generator` package.

# Purpose / Use cases

Costmap generator provides an interface for generating costmap representation of an environment.

# Design

This library is capable of creating costmap with lanelet2 driveable areas applied. For proper operation, an algorithm
should be properly initialized with values, since it doesn't provide any default values. The algorithm takes into account
displacements between costmap, map and vehicle frames position, so providing appropriate transforms is crucial for its
correct operation.

The process of costmap generation consists of the following steps:

1. Costmap generator receives the HAD map, translation between costmap and vehicle and transform between map and costmap.
2. Drivable areas, which are roads, parking spots and parking accesses, are extracted from HAD map and converted to
   polygons.
3. Costmap is being moved to the robot's center position to align its position in map frame.
4. Each layer of costmap is filled. For now, two layers exist:
   * HAD map layer which is created by filling drivable area polygons with 0 cost and the rest with obstacle cost;
   * combined layer where every element is a max of the corresponding elements of all the layers; currently, when only
     one other layer exists, it is just a copy of the HAD map layer.
5. If enabled, the bounding box of the drivable area polygons is calculated and costmap is being trimmed.
6. Result costmap is returned.

## Configuration

| Name                  | Type   | Description                                                    |
| --------------------- | ------ | -------------------------------------------------------------- |
| `use_wayarea`         | bool   | whether using `wayarea` from `~/client/HAD_Map_Service` or not |
| `bound_costmap`       | bool   | whether trim output costmap or not                             |
| `costmap_frame`       | string | created costmap's coordinate                                   |
| `grid_min_value`      | double | minimum cost for gridmap                                       |
| `grid_max_value`      | double | maximum cost for gridmap                                       |
| `grid_resolution`     | double | resolution for gridmap                                         |
| `grid_length_x`       | int    | size of gridmap for x direction                                |
| `grid_length_y`       | int    | size of gridmap for y direction                                |
| `grid_position_x`     | int    | offset from coordinate in x direction                          |
| `grid_position_y`     | int    | offset from coordinate in y direction                          |

# Future extensions / Unimplemented parts

- Handle other sources of environment information like point clouds or detected objects

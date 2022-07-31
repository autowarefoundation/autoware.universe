# Apparent Safe Velocity Limiter

## Purpose

This node reduces the velocity of a trajectory around obstacles in order to convey a better feeling of apparent safety to the passengers.

## Inner-workings / Algorithms

Using a parameter `min_ttc` (minimum time to collision), the feeling of apparent safety is defined as
"no collision with an obstacle even without control inputs for a duration of `min_ttc`".

In this node, the motion of the ego vehicle is simulated at each point of the trajectory to create a corresponding footprint.
If the footprint collides with some obstacle, the velocity at the trajectory point is reduced to remove the collision.

### Simulated Motion, Footprint, and Collision Distance

The motion of the ego vehicle is simulated at each trajectory point using the `heading`, `velocity`, and `steering` defined at the point.
Footprints are then constructed from these simulations and checked for collision.
If a collision is found, the distance from the trajectory point is used to calculate the adjusted velocity that would produce a collision-free footprint. Parameter `simulation.distance_method` allow to switch between an exact distance calculation and a less expensive approximation using a simple euclidian distance.

Two models can be selected with parameter `simulation.model` for simulating the motion of the vehicle: a simple particle model and a more complicated bicycle model.

#### Particle Model

The particle model uses the constant heading and velocity of the vehicle at a trajectory point to simulate the future motion.
The simulated forward motion corresponds to a straight line and the footprint to a rectangle.

##### Footprint

The rectangle footprint is built from 2 lines parallel to the simulated forward motion and at a distance of half the vehicle width.

![particle_footprint_image](./media/particle_footprint.png)

##### Distance

When a collision point is found within the footprint, the distance is calculated as described in the following figure.

![particle_collision_distance_image](./media/particle_distance.png)

#### Bicycle Model

The bicycle model uses a constant heading, velocity, and steering of the vehicle at a trajectory point to simulate the future motion.
The simulated forward motion corresponds to an arc around the circle of curvature associated with the steering.
Uncertainty in the steering can be introduced with the `simulation.steering_offset` parameter which will generate a range of motion from a left-most to a right-most steering.
This results in 3 curved lines starting from the same trajectory point.
A parameter `simulation.nb_points` is used to adjust the precision of these lines, with a minimum of `2` resulting in straight lines and higher values increasing the precision of the curves.

##### Footprint

The footprint of the bicycle model is created from lines parallel to the left and right simulated motion at a distance of half the vehicle width.
In addition, points on the left and right of the end point of the central simulated motion are used to complete the polygon.

![bicycle_footprint_image](./media/bicycle_footprint.png)

##### Distance

The distance to a collision point is calculated by finding the curvature circle passing through the trajectory point and the collision point.

![bicycle_collision_distance_image](./media/bicycle_distance.png)

### Obstacle Detection

For efficient collision detection with a footprint polygon, linestrings along obstacles are created from
3 possible sources: the lanelet map, an occupancy grid, and a pointcloud.
The lanelet map is always checked for obstacles but the other source is switched using parameter `obstacles.dynamic_source`.

Collision detection relies on
[`boost::geometry::intersection`](https://www.boost.org/doc/libs/1_78_0/libs/geometry/doc/html/geometry/reference/algorithms/intersection/intersection_3.html) to find the intersection between an obstacle's linestring and a footprint polygon.

Some dynamic obstacles are ignored and must be masked out from the occupancy grid and pointcloud.
Parameter `dynamic_obstacles_min_vel` sets the velocity above which a dynamic obstacle is masked out.
Parameter `dynamic_obstacles_buffer` sets the extra distance around dynamic obstacles used for masking.

#### Lanelet Map

Information about static obstacles are stored in the Lanelet map in the form of tags associated linestrings.
First, the ego route is used to list all lanelets that the ego may drive through.
The tags of the left and right linestrings from each of these lanelets are then checked.
If any is tagged with one of the value from parameter `obstacles.static_map_tags`, then it will be used as an obstacle.

#### Occupancy Grid

First, dynamic obstacles are masked from occupancy grid in order
to avoid incorrectly detecting collision with vehicle moving at velocities higher than parameter `obstacles.dynamic_obstacles_min_vel`.
Parameter `obstacles.dynamic_obstacles_buffer` is also used to increase the size of the mask and reduce noise.

After the occupancy grid has been converted to an image, a threshold is applied to only keep cells with a value above parameter `obstacles.occupancy_grid_threshold`.

Contours are then extracted from the image and corresponding linestrings are created
the occupancy grid using the opencv function
[`findContour`](https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a).

#### Pointcloud

TODO once finalized

### Velocity Adjustment

If a collision is found, the velocity at the trajectory point is adjusted such that the resulting footprint would no longer collide with an obstacle:
$velocity = \frac{dist\_to\_collision}{min\_ttc}$

To prevent sudden deceleration of the ego vehicle, the parameter `max_deceleration` limits the deceleration relative to the current ego velocity.
For a trajectory point occuring at a duration `t` in the future (calculated from the original velocity profile),
the adjusted velocity cannot be set lower than $v_{current} - t * max\_deceleration$.

Furthermore, a parameter `min_adjusted_velocity`
provides a lower bound on the modified velocity.

### Trajectory preprocessing

The node only modifies part of the input trajectory, starting from the current ego position.
A parameter `start_distance` is used to adjust how far ahead of the ego position the velocities will start being modified.

To reduce computation cost at the cost of precision, the trajectory can be downsampled using parameter `downsample_factor`.
For example a value of `1` means all trajectory points will be evaluated while a value of `10` means only 1/10th of the points will be evaluated.

## Inputs / Outputs

### Input

| Name                          | Type                                             | Description                                        |
| ----------------------------- | ------------------------------------------------ | -------------------------------------------------- |
| `~/input/trajectory`          | `autoware_auto_planning_msgs/Trajectory`         | Reference trajectory                               |
| `~/input/occupancy_grid`      | `nav_msgs/OccupancyGrid`                         | Occupancy grid with obstacle information           |
| `~/input/obstacle_pointcloud` | `sensor_msgs/PointCloud2`                        | Pointcloud containing only obstacle points         |
| `~/input/dynamic_obstacles`   | `autoware_auto_perception_msgs/PredictedObjects` | Dynamic objects                                    |
| `~/input/odometry`            | `nav_msgs/Odometry`                              | Odometry used to retrieve the current ego velocity |
| `~/input/map`                 | `autoware_auto_mapping_msgs/HADMapBin`           | Vector map used to retrieve static obstacles       |
| `~/input/route`               | `autoware_auto_mapping_msgs/HADMapRoute`         | Route taken by ego on the map                      |

### Output

| Name                     | Type                                     | Description                                  |
| ------------------------ | ---------------------------------------- | -------------------------------------------- |
| `~/output/trajectory`    | `autoware_auto_planning_msgs/Trajectory` | Trajectory with adjusted velocities          |
| `~/output/debug_markers` | `visualization_msgs/MarkerArray`         | Debug markers (envelopes, obstacle polygons) |

## Parameters

| Name                                  | Type        | Description                                                                                                                             |
| ------------------------------------- | ----------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `min_ttc`                             | float       | [s] required minimum time with no collision at each point of the trajectory assuming constant heading and velocity.                     |
| `distance_buffer`                     | float       | [m] required distance buffer with the obstacles.                                                                                        |
| `min_adjusted_velocity`               | float       | [m/s] minimum adjusted velocity this node can set.                                                                                      |
| `max_deceleration`                    | float       | [m/s²] maximum deceleration an adjusted velocity can cause.                                                                             |
| `start_distance`                      | float       | [m] controls from which part of the trajectory (relative to the current ego pose) the velocity is adjusted.                             |
| `downsample_factor`                   | int         | trajectory downsampling factor to allow tradeoff between precision and performance.                                                     |
| `simulation.model`                    | string      | model to use for forward simulation. Either "particle" or "bicycle".                                                                    |
| `simulation.distance_method`          | string      | method to use for calculating distance to collision. Either "exact" or "approximation".                                                 |
| `simulation.steering_offset`          | float       | offset around the steering used by the bicycle model.                                                                                   |
| `simulation.nb_points`                | int         | number of points used to simulate motion with the bicycle model.                                                                        |
| `obstacles.dynamic_source`            | string      | source of dynamic obstacle used for collision checking. Can be "occupancy_grid", "point_cloud", or "static_only" (no dynamic obstacle). |
| `obstacles.occupancy_grid_threshold`  | int         | value in the occupancy grid above which a cell is considered an obstacle.                                                               |
| `obstacles.dynamic_obstacles_buffer`  | float       | buffer around dynamic obstacles used when masking an obstacle in order to prevent noise.                                                |
| `obstacles.dynamic_obstacles_min_vel` | float       | velocity above which to mask a dynamic obstacle.                                                                                        |
| `obstacles.static_map_tags`           | string list | linestring of the lanelet map with this tags are used as obstacles.                                                                     |
| `obstacles.filter_envelope`           | bool        | wether to use the safety envelope to filter the dynamic obstacles source.                                                               |

## Assumptions / Known limits

The velocity profile produced by this node is not meant to be a realistic velocity profile
and can contain sudden jumps of velocity with no regard for acceleration and jerk.
This velocity profile is meant to be used as an upper bound on the actual velocity of the vehicle.

## (Optional) Error detection and handling

The critical case for this node is when an obstacle is falsely detected very close to the trajectory such that
the corresponding apparent safe velocity suddenly becomes very low.
This can cause a sudden brake and two mechanisms can be used to mitigate these errors.

Parameter `min_adjusted_velocity` allow to set a minimum to the adjusted velocity, preventing the node to slow down the vehicle too much.
Parameter `max_deceleration` allow to set a maximum deceleration (relative to the _current_ ego velocity) that the adjusted velocity would incur.

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

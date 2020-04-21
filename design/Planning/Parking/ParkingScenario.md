# Parking Scenario
This scenario is meant to be used to plan manuevers to park vehicle in parking space. Compared to LaneDrivingScenario, this scenario has relative less constraints about the shape of trajectory.

## Requirements:
Lane Driving Scenario must satisfy the following use cases:
* Park vehicle in parking space

For the details about related requirements, please refer to the [document for Planning stack](/design/Planning/Planning.md).


### Input
- Route: `autoware_planning_msgs::Route` <br> This includes the final goal pose and which lanes are available for trajectory planning, but only goal pose is used for planning.
- Map: `autoware_lanelet_msgs::MapBin` <br> This provides all static information about the environment. This is meant to be used to generate drivable area.
- Dynamic Objects: `autoware_perception_msgs::DynamicObjectArray` <br> This provides all obstacle information calculated from sensors. Scenario module should calculate trajectory such that vehicle does not collide with other objects. This can be either done by planning velocity so that it stops before hitting obstacle, or by calculate path so that vehicle avoids the obstacle.
- Scenario: `autoware_planning_msgs::Scenario` <br> This is the message from scenario selector. All modules only run when Parking scenario is selected by this topic.

### Outputs
- Trajectory: `autoware_planning_msgs::Trajectory` <br> This contains trajectory that Control must follow. The shape and velocity of the trajectory must satisfy all the requirements.

## Design
![ParkingScenario.png](/design/img/ParkingScenario.png)

### Costmap Generator
This gives spacial constraints to freespace planner.
#### Input
- Map: `autoware_lanelet_msgs::MapBin` <br> This provides all static information about the environment. This is meant to be used to generate drivable area.
- Dynamic Objects: `autoware_perception_msgs::DynamicObjectArray` <br> This provides all obstacle information calculated from sensors. Scenario module should calculate trajectory such that vehicle does not collide with other objects. This can be either done by planning velocity so that it stops before hitting obstacle, or by calculate path so that vehicle avoids the obstacle.
- Scenario: `autoware_planning_msgs::Scenario` <br> This is the message from scenario selector. All modules only run when Parking scenario is selected by this topic.

#### Output
* Costmap: `nav_msgs::OccupancyGrid.msg`<br> This contains spaces that can be used for trajectory planning. The grid is considered not passable(occupied) if it is outside of parking lot polygon specified map, and percepted objects lie on the grid.

### Freespace Planner
Freespace planner calculates trajectory that navigates vehicle to goal pose given by route. It must consider vehicle's kinematic model.

#### Input
- Route: `autoware_planning_msgs::Route` <br> This includes the final goal pose and which lanes are available for trajectory planning, but only goal pose is used for planning.
- Vehicle Pose: `tf_msgs::tf`
- Vehicle Velocity: `geometry_msgs::Twist`

#### Output
- Trajectory: `autoware_planning_msgs::Trajectory` <br> This contains trajectory that Control must follow. The shape and velocity of the trajectory must satisfy all the requirements.

# Planning

## Overview

Planning stack acts as the “brain” of autonomous driving. It uses all the results from Localization, Perception, and Map stacks to decide its maneuver and gives final trajectory to Control stack.

## Role

These are high-level roles of Planning stack:

- Calculates route that navigates to desired goal
- Plans trajectory to follow the route
  - Make sure that vehicle does not collide with obstacles, including pedestrians and other vehicles)
  - Make sure that the vehicle follows traffic rules during the navigation. This includes following traffic light, stopping at stoplines, stopping at crosswalks, etc.
- Plan sequences of trajectories that is feasible for the vehicle. (e.g. no sharp turns that is kinematically impossible)

## Use Cases

Planning stack must satisfy following use cases:

1. Navigate vehicle from start to goal
2. Operating lane change
3. Driving along lane
4. Following speed limit of lane
5. Follow traffic light
6. Follow yield/stop signs
7. Turning left/right at intersections
8. Park vehicle at parking space (either reverse parking or forward first parking)

## Requirements

1. **Planning route from start to goal** (Use Case 1)

   - Planning stack should be able to get starting lane and goal lane from given start pose and goal pose either in earth frame or map frame
   - Planning stack should be able to calculate sequences of lanes that navigates vehicle from start lane to goal lane that minimizes cost function(either time based cost or distance based cost)

2. **Driving along lane** (Use Case 2)

   - Vehicle must drive between left boundary and right boundary of driving lane
   - The vehicle must have at least 2 seconds margin between other vehicles so that it has enough distance to stop without collision. [reference](https://www.cedr.eu/download/Publications/2010/e_Distance_between_vehicles.pdf)

3. **Operating lane change** (Use Case 3)

   - Vehicle must change lane when
     - lane change is necessary to follow planned route
     - If current driving lane is blocked (e.g. by parked vehicle)
   - Vehicle must turn on appropriate turn signal 3 seconds before lane change and it must be turned on until lane change is finished
   - Vehicle should stay in lane at least for 3 second before operating lane change for other participants to recognize ego vehicle's turn signal.
   - there must be 2 seconds margin between any other vehicles during lane change
   - lane change finishes 30m before any intersections
   - vehicle should abort lane change when all of the following conditions are satisfied:
     - Vehicle(base_link) is still in the original lane
     - there is no longer 2 seconds margin between other n vehicles during lane change e.g. due to newly detected vehicles

4. **Follow speed limit of lane** (Use Case 4)

   - Speed profile of trajectory points in a lane must be below speed limit of the lane.

5. **Follow traffic light** (Use Case 5)

   - Planning stack should refer to Perception output of the traffic light associated to driving lane.
   - Speed profile of a trajectory at the associated stopline must be zero when relevant traffic light is red and it has enough distance to stop before the stopline with given deceleration configuration

6. **Turning left/right at intersections** (Use Case 6)

   - Vehicle must stop before entering intersection whenever other vehicles are entering intersection unless ego vehicle has right of way

7. **Parking** (Use Case 7)

   - Vehicle must not hit other vehicle, curbs, or other obstacle during parking
     - i.e. All points in planned trajectory has enough distance from other objects with ego vehicle's footprint taken into account

8. **General requirements to trajectory**
   - Planned trajectory must satisfy requirements from Control stack:
     - Planned trajectory must have speed profile that satisfies given acceleration and jerk limits unless vehicle is under emergency e.g. when pedestrian suddenly jumps into driving lane or front vehicle suddenly stops.
     - Planned trajectory must be feasible by the given vehicle kinematic model
     - Planned trajectory must satisfy given lateral acceleration and jerk limit
     - Planned trajectory points within _n_ [m] from ego vehicle should not change over time unless sudden steering or sudden acceleration is required to avoid collision with other vehicles.
       - _n_[m] = _velocity_of_ego_vehicle_\*_configured_time_horizon_

## Input

The table below summarizes the overall input into Planning stack:

| Input                           | Topic Name(Data Type)                                                                                                   | Explanation                                                                                                                                                                                                                                                                                                         |
| ------------------------------- | ----------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Vehicle Pose                    | `/tf (map->base_link)`<br>(`tf::tfMessage`)                                                                             | Planning requires vehicle pose in map frame, which is the frame where all planning takes place.                                                                                                                                                                                                                     |
| Vehicle Velocity                | `/localization/twist`<br>(`geometry_msgs::Twist`)                                                                       | This includes vehicle's velocity information. It is used to predict future pose on trajectory to detect collision with other objects.                                                                                                                                                                               |
| Map data                        | `/map/vector_map`<br>(`autoware_lanelet2_msgs::LaneletMapBin`)                                                          | This includes all static information about the environment, such as: <ul><li>Lane connection information used for route planning from starting position to goal position</li><li>Lane geometry to generate reference path used to calculate trajectory </li><li> All information related to traffic rules</li></ul> |
| Detected Obstacle Information   | `/perception/object_recognition/objects`<br>(`autoware_planning_msgs::DynamicObjectsArray`)                             | This includes information that cannot be known beforehand such as pedestrians and other vehicles. Planning stack will plan maneuvers to avoid collision with such objects.                                                                                                                                          |
| Goal position                   | `/planning/goal_pose`<br>(`geometry_msgs::PoseStamped`)                                                                 | This is the final pose that Planning stack will try to achieve.                                                                                                                                                                                                                                                     |
| TrafficLight recognition result | `/perception/traffic_light_recognition/traffic_light_states`<br>(`autoware_traffic_light_msgs::TrafficLightStateArray`) | This is the real time information about the state of each traffic light. Planning stack will extract the one that is relevant to planned path and use it to decide whether to stop at intersections.                                                                                                                |

## Output

The table below summarizes the final output from Planning stack:

| Output      | Topic(Data Type)                                                    | Explanation                                                                                                                                                |
| ----------- | ------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Trajectory  | `/planning/trajectory`<br>(`autoware_planning_msgs::Trajectory`)    | This is the sequence of pose that Control stack must follow. This must be smooth, and kinematically possible to follow by the Control stack.               |
| Turn Signal | `/vehicle/turn_signal_cmd`<br>(`autoware_vehicle_msgs::TurnSignal`) | This is the output to control turn signals of the vehicle. Planning stack will make sure that turn signal will be turned on according to planned maneuver. |

## Design

In order to achieve the requirements stated above, Planning stack is decomposed into the diagram below.
Each requirement is met in following modules:

- Requirement 1: Mission calculates the overall route to reach goal from starting position
- Requirement 2-7: LaneDriving scenario plans trajectory along lanes in planned route
- Requirement 8: Parking scenario plans trajectory in free space to park into parking space
- Requirement 9: Both LaneDriving and Parking should output trajectory that satisfies the requirement

We have looked into different autonomous driving stacks and concluded that it is technically difficult to use unified planner to handle every possible situation. (See [here](DesignRationale.md) for more details). Therefore, we have decided to set different planners in parallel dedicated for each use case, and let scenario selector to decide depending on situations. Currently, we have reference implementation with two scenarios, on-road planner and parking planner, but any scenarios (e.g. highway, in-emergency, etc.) can be added as needed.

It may be controversial whether new scenario is needed or existing scenario should be enhanced when adding new feature, and we still need more investigation to set the definition of “Scenario” module.

![Planning_component](image/PlanningOverview.svg)

## Scenarios

### Role

The role of Scenario module is to calculate trajectory message from route message. It should only plan when the module is selected by the scenario selector module. This is where all behavior planning is done.

### Input

- Route: `autoware_planning_msgs::Route` <br> This includes the final goal pose and which lanes are available for trajectory planning.
- Map: `autoware_lanelet_msgs::MapBin` <br> This provides all static information about the environment, including lane connection, lane geometry, and traffic rules. Scenario module should plan trajectory such that vehicle follows all traffic rules specified in map.
- Dynamic Objects: `autoware_perception_msgs::DynamicObjectArray` <br> This provides all obstacle information calculated from sensors. Scenario module should calculate trajectory such that vehicle does not collide with other objects. This can be either done by planning velocity so that it stops before hitting obstacle, or by calculate path so that vehicle avoids the obstacle.
- Scenario: `autoware_planning_msgs::Scenario` <br> This is the message from scenario selector. Scenario modules only run when the module is selected by this topic.

### Output

- Trajectory: `autoware_planning_msgs::Trajectory` <br> This contains trajectory that Control must follow. The shape and velocity of the trajectory must satisfy all the use cases for the scenario module.
- Turn Signal: `autoware_vehicle_msgs::TurnSignal` <br> Turn signal command should also be published because Scenario module is only aware of the traffic rules and operating maneuvers in the whole Autoware stack.

## Reference Implementation

The reference implementation of the planning module in the latest version is shown as below.
![reference-implementation](image/autoware-iv-planning-v1.0.0.main.drawio.svg)

<!-- ![reference-implementation](image/autoware-iv-planning-v0.13.0.develop.drawio.svg) -->

For more details, please refer to the design documents in each package.

- [_mission_planner_](https://tier4.github.io/autoware.iv/tree/main/planning/mission_planning/mission_planner/mission_planner-design/): calculate route from start to goal based on the map information.
- _lane_change_planner_: execute lane change.
- [_behavior_velocity_planner_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/): calculates max speed based on the traffic rules.
  - [_detection_area_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/#detection-area)
  - [_blind_spot_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/#blind-spot)
  - [_cross_walk_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/#crosswalk)
  - [_stop_line_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/#stop-line)
  - [_traffic_light_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/#traffic-light)
  - [_intersection_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/#intersection)
  <!-- - [_occlusion_spot_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/#occlusion-spot) -->
- [_obstacle_avoidance_planner_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/obstacle_avoidance_planner-design.ja/): calculate path shape under obstacle and drivable area constraints
- [_surround_obstacle_checker_](https://tier4.github.io/autoware.iv/tree/main/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker/surround_obstacle_checker-design/): keeps the vehicle being stopped when there are obstacles around the ego-vehicle. It works only when the vehicle is stopped.
- [_obstacle_stop_planner_](https://github.com/tier4/obstacle_stop_planner_refine/tree/main/obstacle_stop_planner_refine): (NOTE: link is temporal) When there are obstacles on or near the trajectory, it calculates the maximum velocity of the trajectory points depending on the situation: stopping, slowing down, or adaptive cruise (following the car).
  - [_stop_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/#obstacle-stop-planner_1)
  - [_slow_down_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/#slow-down-planner)
  - [_adaptive_cruise_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/#adaptive-cruise-controller)
- [_costmap_generator_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/parking/costmap_generator/): generates a costmap for path generation from dynamic objects and lane information.
- [_freespace_planner_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/parking/freespace_planner/): calculates trajectory considering the feasibility (e.g. curvature) for the freespace scene.
- [_scenario_selector_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/scenario_selector/) : chooses a trajectory according to the current scenario.
- [_motion_velocity_smoother_](https://tier4.github.io/autoware.iv/tree/develop/planning/scenario_planning/common/motion_velocity_smoother/motion_velocity_smoother-design/): calculates final velocity considering velocity, acceleration, and jerk constraints.

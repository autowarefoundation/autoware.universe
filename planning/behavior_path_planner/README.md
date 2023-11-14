# Behavior Path Planner

The Behavior Path Planner's main objective is to significantly enhances the safety of autonomous vehicles by minimizing the risk of accidents. It improves driving efficiency through time conservation and underpins reliability with its rule-based approach. Additionally, it allows users to integrate their own custom behavior modules or use it with different types of vehicles, such as cars, buses, and delivery robots, as well as in various environments, from busy urban streets to open highways.

The module begins by thoroughly analyzing the ego vehicle's current situation, including position, speed, and surrounding environment, leading to essential driving decisions about lane changes or stopping, and subsequently generates a path that is both safe and efficient, considering road geometry, traffic rules, and dynamic conditions, while also incorporating obstacle avoidance to respond to static and dynamic obstacles such as other vehicles, pedestrians, or unexpected roadblocks, ensuring safe navigation.

Moreover, the planner actively interacts with other traffic participants, predicting their actions and accordingly adjusting the vehicle's path. This ensures not only the safety of the autonomous vehicle but also contributes to smooth traffic flow. Its adherence to traffic laws, including speed limits and traffic signals, further guarantees lawful and predictable driving behavior. The planner is also designed to minimize sudden or abrupt maneuvers, aiming for a comfortable and natural driving experience.

## Purpose / Use Cases

Essentially, the module has three primary responsibilities:

1. Creating a **path based** on the traffic situation.
2. Generating **drivable area**, i.e. the area within which the vehicle can maneuver.
3. Generating **turn signal** commands to be relayed to the vehicle interface.

## Features

### Supported Scene Modules

Behavior Path Planner has following scene modules

| Name                     | Description                                                                                                                                                                | Details                                                                 |
| :----------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------- |
| Lane Following           | this module generates reference path from lanelet centerline.                                                                                                              | LINK                                                                    |
| Avoidance                | this module generates avoidance path when there is objects that should be avoid.                                                                                           | [LINK](./docs/behavior_path_planner_avoidance_design.md)                |
| Dynamic Avoidance        | WIP                                                                                                                                                                        | LINK                                                                    |
| Avoidance By Lane Change | this module generates lane change path when there is objects that should be avoid.                                                                                         | [LINK](./docs/behavior_path_planner_avoidance_by_lane_change_design.md) |
| Lane Change              | this module is performed when it is necessary and a collision check with other vehicles is cleared.                                                                        | [LINK](./docs/behavior_path_planner_lane_change_design.md)              |
| External Lane Change     | WIP                                                                                                                                                                        | LINK                                                                    |
| Start Planner            | this module is performed when ego-vehicle is in the road lane and goal is in the shoulder lane. ego-vehicle will stop at the goal.                                         | [LINK](./docs/behavior_path_planner_goal_planner_design.md)             |
| Goal Planner             | this module is performed when ego-vehicle is stationary and footprint of ego-vehicle is included in shoulder lane. This module ends when ego-vehicle merges into the road. | [LINK](./docs/behavior_path_planner_start_planner_design.md)            |
| Side Shift               | (for remote control) shift the path to left or right according to an external instruction.                                                                                 | [LINK](./docs/behavior_path_planner_side_shift_design.md)               |

!!! Note

    click on the following images to view the video of their execution

    <div align="center">
        <table>
            <tr>
                <td><img src="./image/supported_module_lane_following.svg" alt="Lane Following Module" width="300"></td>
                <td><a href="https://www.youtube.com/watch?v=A_V9yvfKZ4E"><img src="./image/supported_module_avoidance.svg" alt="Avoidance Module" width="300"></a></td>
                <td><img src="./image/supported_module_avoidance_by_lane_change.svg" alt="Avoidance by Lane Change Module" width="300"></td>
            </tr>
            <tr>
                <td><a href="https://www.youtube.com/watch?v=0jRDGQ84cD4"><img src="./image/supported_module_lane_change.svg" alt="Lane Change Module" width="300"></a></td>
                <td><a href="https://www.youtube.com/watch?v=xOjnPqoHup4"><img src="./image/supported_module_start_planner.svg" alt="Start Planner Module" width="300"></a></td>
                <td><a href="https://www.youtube.com/watch?v=ornbzkWxRWU"><img src="./image/supported_module_goal_planner.svg" alt="Goal Planner Module" width="300"></a></td>
            </tr>
        </table>
    </div>

!!! Note

    Users can refer to [Planning component design](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/#supported-functions) for some additional behavior.

#### How to add or implement new module?

All scene modules are implemented by inheriting base class `scene_module_interface.hpp`.

!!! Warning

    The remainder of this subsection is work in progress (WIP).

### Planner Manager

The Planner Manager's responsibilities include:

1. Activating the relevant scene module in response to the specific situation faced by the autonomous vehicle. For example, when a parked vehicle block ego vehicle's driving lane, the manager would engage the avoidance module.
2. Managing the execution order of when multiple modules are running at the same time. If, for instance, both lane-changing and avoidance modules are operational, the manager decides which should take precedence.
3. When multiple modules are activated simultaneously and each generates its own path, the manager merges these into a single functional path.

!!! note

    To check the scene module's transition, i.e.: registered, approved and candidate modules, set `verbose: true` in the [behavior path planner configuration file](https://github.com/autowarefoundation/autoware_launch/blob/0cd5d891a36ac34a32a417205905c109f2bafe7b/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml#L3).

    ![Scene module's transition table](./image/checking_module_transition.png)

!!! note

    For more in-depth information, refer to [Manager design](./docs/behavior_path_planner_manager_design.md) document.

## Inputs / Outputs / API

### Input

| Name                          | Required? | Type                                                   | Description                                                                                                                                                                                                              |
| :---------------------------- | :-------: | :----------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ~/input/odometry              |     ○     | `nav_msgs::msg::Odometry`                              | for ego velocity.                                                                                                                                                                                                        |
| ~/input/accel                 |     ○     | `geometry_msgs::msg::AccelWithCovarianceStamped`       | for ego acceleration.                                                                                                                                                                                                    |
| ~/input/objects               |     ○     | `autoware_auto_perception_msgs::msg::PredictedObjects` | dynamic objects from perception module.                                                                                                                                                                                  |
| ~/input/occupancy_grid_map    |     ○     | `nav_msgs::msg::OccupancyGrid`                         | occupancy grid map from perception module. This is used for only Goal Planner module.                                                                                                                                    |
| ~/input/traffic_signals       |     ○     | `autoware_perception_msgs::msg::TrafficSignalArray`    | traffic signals information from the perception module                                                                                                                                                                   |
| ~/input/vector_map            |     ○     | `autoware_auto_mapping_msgs::msg::HADMapBin`           | vector map information.                                                                                                                                                                                                  |
| ~/input/route                 |     ○     | `autoware_auto_mapping_msgs::msg::LaneletRoute`        | current route from start to goal.                                                                                                                                                                                        |
| ~/input/scenario              |     ○     | `tier4_planning_msgs::msg::Scenario`                   | Launches behavior path planner if current scenario == `Scenario:LaneDriving`.                                                                                                                                            |
| ~/input/lateral_offset        |     △     | `tier4_planning_msgs::msg::LateralOffset`              | lateral offset to trigger side shift                                                                                                                                                                                     |
| ~/system/operation_mode/state |     ○     | `autoware_adapi_v1_msgs::msg::OperationModeState`      | Allows planning module to know if vehicle is in autonomous mode or can be controlled<sup>[ref](https://github.com/autowarefoundation/autoware.universe/blob/main/system/default_ad_api/document/operation-mode.md)</sup> |

- ○ Mandatory: Planning Module would not work if anyone of this is not present.
- △ Optional: Some module would not work, but Planning Module can still be operated.

### Output

| Name                          | Type                                                     | Description                                                                                    | QoS Durability    |
| :---------------------------- | :------------------------------------------------------- | :--------------------------------------------------------------------------------------------- | ----------------- |
| ~/output/path                 | `autoware_auto_planning_msgs::msg::PathWithLaneId`       | the path generated by modules.                                                                 | `volatile`        |
| ~/output/turn_indicators_cmd  | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand` | turn indicators command.                                                                       | `volatile`        |
| ~/output/hazard_lights_cmd    | `tier4_planning_msgs::msg::PathChangeModuleArray`        | hazard lights command.                                                                         | `volatile`        |
| ~/output/modified_goal        | `autoware_planning_msgs::msg::PoseWithUuidStamped`       | output modified goal commands.                                                                 | `transient_local` |
| ~/output/stop_reasons         | `tier4_planning_msgs::msg::StopReasonArray`              | output modified goal commands.                                                                 | `volatile`        |
| ~/output/reroute_availability | `tier4_planning_msgs::msg::RerouteAvailability`          | the path the module is about to take. to be executed as soon as external approval is obtained. | `volatile`        |

### Debug

| Name                                    | Type                                                     | Description                                                                                    | QoS Durability |
| :-------------------------------------- | :------------------------------------------------------- | :--------------------------------------------------------------------------------------------- | -------------- |
| ~/debug/avoidance_debug_message_array   | `autoware_auto_planning_msgs::msg::PathWithLaneId`       | the path generated by modules.                                                                 | `volatile`     |
| ~/debug/lane_change_debug_message_array | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand` | turn indicators command.                                                                       | `volatile`     |
| ~/debug/maximum_drivable_area           | `tier4_planning_msgs::msg::StopReasonArray`              | output modified goal commands.                                                                 | `volatile`     |
| ~/debug/turn_signal_info                | `tier4_planning_msgs::msg::PathChangeModuleArray`        | hazard lights command.                                                                         | `volatile`     |
| ~/debug/bound                           | `autoware_planning_msgs::msg::PoseWithUuidStamped`       | output modified goal commands.                                                                 | `volatile`     |
| ~/planning/path_candidate               | `tier4_planning_msgs::msg::RerouteAvailability`          | the path the module is about to take. to be executed as soon as external approval is obtained. | `volatile`     |
| ~/planning/path_reference               | `tier4_planning_msgs::msg::RerouteAvailability`          | the path the module is about to take. to be executed as soon as external approval is obtained. | `volatile`     |

!!! note

    For specific information of which topics are being subscribed and published, refer to [behavior_path_planner.xml](https://github.com/autowarefoundation/autoware.universe/blob/9000f430c937764c14e43109539302f1f878ed70/planning/behavior_path_planner/launch/behavior_path_planner.launch.xml#L36-L49).

## How to enable or disable the modules

## Generating Drivable Area

### Static Drivable area logic

The drivable area is used to determine the area in which the ego vehicle can travel. The primary goal of static drivable area expansion is to ensure safe travel by generating an area that encompasses only the necessary spaces for the vehicle's current behavior, while excluding non-essential areas. For example, while `avoidance` module is running, the drivable area includes additional space needed for maneuvers around obstacles, and it limits the behavior by not extending the avoidance path outside of lanelet areas.

<div align="center">
    <table>
        <tr>
            <td><img src="./image/static_drivable_area_before_expansion.png" alt="Before expansion"></td>
        </tr>
        <tr>
            <td><img src="./image/static_drivable_area_after_expansion.png" alt="After expansion"></td>
        </tr>
    </table>
</div>

Static drivable area expansion operates under assumptions about the correct arrangement of lanes and the coverage of both the front and rear of the vehicle within the left and right boundaries. Key parameters for drivable area generation include extra footprint offsets for the ego vehicle, the handling of dynamic objects, maximum expansion distance, and specific methods for expansion. Additionally, since each module generates its own drivable area, before passing it as the input to generate the next running module's drivable area, or before generating a unified drivable area, the system sorts drivable lanes based on the vehicle's passage order. This ensures the correct definition of the lanes used in drivable area generation.

!!! note

    Click [here](./docs/behavior_path_planner_drivable_area_design.md) for details.

### Dynamic Drivable Area Algorithm

Large vehicles require much more space, which sometimes causes them to veer out of their current lane. A typical example being a bus making a turn at a corner. In such cases, relying on a static drivable area is insufficient, since the static method depends on lane information provided by high-definition maps. To overcome the limitations of the static approach, the dynamic drivable area expansion algorithm adjusts the navigable space for an autonomous vehicle in real-time. It conserves computational power by reusing previously calculated path data, updating only when there is a significant change in the vehicle's position. The system evaluates the minimum lane width necessary to accommodate the vehicle's turning radius and other dynamic factors. It then calculates the optimal expansion of the drivable area's boundaries to ensure there is adequate space for safe maneuvering, taking into account the vehicle's path curvature. The rate at which these boundaries can expand or contract is moderated to maintain stability in the vehicle's navigation. The algorithm aims to maximize the drivable space while avoiding fixed obstacles and adhering to legal driving limits. Finally, it applies these boundary adjustments and smooths out the path curvature calculations to ensure a safe and legally compliant navigable path is maintained throughout the vehicle's operation.

!!! note

    The feature can be enabled in the [drivable_area_expansion.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/0cd5d891a36ac34a32a417205905c109f2bafe7b/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml#L10).

## Generating Turn Signal

The Behavior Path Planner module uses the `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand` to output turn signal commands (see [TurnIndicatorsCommand.idl](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand.idl)). The system evaluates the driving context and determines when to activate turn signals based on its maneuver planning—like turning, lane changing, or obstacle avoidance.

Within this framework, the system differentiates between **desired** and **required** blinker activations. **Desired** activations are those recommended by traffic laws for typical driving scenarios, such as signaling before a lane change or turn. **Required** activations are those that are deemed mandatory for safety reasons, like signaling an abrupt lane change to avoid an obstacle.

The `TurnIndicatorsCommand` message structure has a command field that can take one of several constants: `NO_COMMAND` indicates no signal is necessary, `DISABLE` to deactivate signals, `ENABLE_LEFT` to signal a left turn, and `ENABLE_RIGHT` to signal a right turn. The Behavior Path Planner sends these commands at the appropriate times, based on its rules-based system that considers both the **desired** and **required** scenarios for blinker activation.

!!! note

    For more in-depth information, refer to [Turn Signal Design](./docs/behavior_path_planner_turn_signal_design.md) document.

## Generating Shifted Path

A sophisticated methodology is used for path generation, particularly focusing on maneuvers like lane changes and avoidance. At the core of this design is the smooth lateral shifting of the reference path, achieved through a constant-jerk profile. This approach ensures a consistent rate of change in acceleration, facilitating smooth transitions and minimizing abrupt changes in lateral dynamics, crucial for passenger comfort and safety.

The design involves complex mathematical formulations for calculating the lateral shift of the vehicle's path over time. These calculations include determining lateral displacement, velocity, and acceleration, while considering the vehicle's lateral acceleration and velocity limits. This is essential for ensuring that the vehicle's movements remain safe and manageable.

The `ShiftLine` struct (see [here](https://github.com/autowarefoundation/autoware.universe/blob/9000f430c937764c14e43109539302f1f878ed70/planning/behavior_path_planner/include/behavior_path_planner/utils/path_shifter/path_shifter.hpp#L35-L48) for the actual data structure) is utilized to represent points along the path where the lateral shift starts and ends. It includes details like the start and end points in absolute coordinates, the relative shift lengths at these points compared to the reference path, and the associated indexes on the reference path. This struct is integral to managing the path shifts, as it allows the path planner to dynamically adjust the trajectory based on the vehicle's current position and planned maneuver.

Furthermore, the design and its implementation incorporate various equations and mathematical models to calculate essential parameters for the path shift. These include the total distance of lateral shift, the maximum allowable lateral acceleration and jerk, and the total time required for the shift. Practical considerations are also noted, such as simplifying assumptions in the absence of a specific time interval for most lane change and avoidance cases.

The shifted path generation logic allows behavior path planner to dynamically generate safe and efficient paths, precisely controlling the vehicle’s lateral movements to ensure smooth execution of lane changes and avoidance maneuvers. This careful planning and execution adhere to the vehicle's dynamic capabilities and safety constraints, maximizing efficiency and safety in autonomous vehicle navigation.

!!! note

    If you're a math lover, refer to [Path Generation Design](./docs/behavior_path_planner_path_generation_design.md) for the nitty-gritty.

## Limitations & Future Work

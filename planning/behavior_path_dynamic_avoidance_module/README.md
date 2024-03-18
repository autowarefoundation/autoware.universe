# Dynamic avoidance design

This module is under development.

## Purpose / Role

This module provides avoidance functions for vehicles, pedestrians, and obstacles in the vicinity of the ego's path in combination with the obstacle_avoidance module.
Each module performs the following roles.
Dynamic Avoidance module: This module cuts off the drivable area according to the position and velocity of the target to be avoided.
Obstacle Avoidance module: This module modifies the path to be followed so that it fits within the drivable area received.

Avoidance functions are also provided by the Avoidance module, which allows avoidance through the outside of own lanes but not against moving objects.
On the other hand, this module can avoid moving objects.
For this reason, the word "dynamic" is used in its name.
The table below lists the avoidance modules that can be used for each situation.

|                          |                         avoid within the own lane                          | avoid through the outside of own lanes |
| :----------------------- | :------------------------------------------------------------------------: | :------------------------------------: |
| avoid not-moving objects | Avoidance Module <br> Dynamic Avoidance Module + Obstacle Avoidance Module |            Avoidance Module            |
| avoid moving objects     |            Dynamic Avoidance Module + Obstacle Avoidance Module            |     No Module (Under Development)      |

## Policy of Algorithms
Here, we describe the policy of inner algorithms.
The inner algorithms can be separated into two parts: The first decide whether to avoid the obstacles and the second cuts off the drivable area against the corresponding obstacle.
If you are interested in more details, please see the code itself.

### Select obstacles to avoid

To decide whether to avoid an object, both the predicted path and the state (poes and twist) of each object are used.
The type of objects the user wants this module to avoid is also required.
Using this information, the module makes a decision to "avoid" objects that "obstruct own passage" and "can be avoided".

As logics for determining whether an object is an obstacle or not, the followings are implemented.
- longtitudinal speed
- 経路に近い 干渉する
- 干渉する時刻が遠すぎると決断を先延ばしにする
- カットアウト
   

避けられるか否か
    交差 lateral speed
    干渉する時刻が近すぎると回避は諦める
    横G横ジャークの制約をみたして避けられるか
    カットイン

どうよけるか 右 左？


The dynamics obstacles meeting the following condition will be avoided.

- The type is designated as `target_object.*`.
- The norm of the obstacle's velocity projected to the ego's path is smaller than `target_object.min_obstacle_vel`.
- The obstacle is in the next lane to the ego's lane, which will not cut-into the ego's lane according to the highest-prioritized predicted path.

### Drivable area modification

交差物体を右か左どちらによけるか


To realize dynamic obstacles for avoidance, the time dimension should be take into an account considering the dynamics.
However, it will make the planning problem much harder to solve.
Therefore, we project the time dimension to the 2D pose dimension.

Currently, the predicted paths of predicted objects are not so stable.
Therefore, instead of using the predicted paths, we assume that the obstacle will run parallel to the ego's path.

First, a maximum lateral offset to avoid is calculated as follows.
The polygon's width to extract from the drivable area is the obstacle width and double `drivable_area_generation.lat_offset_from_obstacle`.
We can limit the lateral shift offset by `drivable_area_generation.max_lat_offset_to_avoid`.

![drivable_area_extraction_width](./image/drivable_area_extraction_width.drawio.svg)

Then, extracting the same directional and opposite directional obstacles from the drivable area will work as follows considering TTC (time to collision).
Regarding the same directional obstacles, obstacles whose TTC is negative will be ignored (e.g. The obstacle is in front of the ego, and the obstacle's velocity is larger than the ego's velocity.).

Same directional obstacles
![same_directional_object](./image/same_directional_object.svg)

Opposite directional obstacles
![opposite_directional_object](./image/opposite_directional_object.svg)

## Example


## Future works

歩行者の回避
より大きな回避幅での回避


## Parameters

| Name                                                                  | Unit  | Type   | Description                                                | Default value |
| :-------------------------------------------------------------------- | :---- | :----- | :--------------------------------------------------------- | :------------ |
| target_object.car                                                     | [-]   | bool   | The flag whether to avoid cars or not                      | true          |
| target_object.truck                                                   | [-]   | bool   | The flag whether to avoid trucks or not                    | true          |
| ...                                                                   | [-]   | bool   | ...                                                        | ...           |
| target_object.min_obstacle_vel                                        | [m/s] | double | Minimum obstacle velocity to avoid                         | 1.0           |
| drivable_area_generation.lat_offset_from_obstacle                     | [m]   | double | Lateral offset to avoid from obstacles                     | 0.8           |
| drivable_area_generation.max_lat_offset_to_avoid                      | [m]   | double | Maximum lateral offset to avoid                            | 0.5           |
| drivable_area_generation.overtaking_object.max_time_to_collision      | [s]   | double | Maximum value when calculating time to collision           | 3.0           |
| drivable_area_generation.overtaking_object.start_duration_to_avoid    | [s]   | double | Duration to consider avoidance before passing by obstacles | 4.0           |
| drivable_area_generation.overtaking_object.end_duration_to_avoid      | [s]   | double | Duration to consider avoidance after passing by obstacles  | 5.0           |
| drivable_area_generation.overtaking_object.duration_to_hold_avoidance | [s]   | double | Duration to hold avoidance after passing by obstacles      | 3.0           |
| drivable_area_generation.oncoming_object.max_time_to_collision        | [s]   | double | Maximum value when calculating time to collision           | 3.0           |
| drivable_area_generation.oncoming_object.start_duration_to_avoid      | [s]   | double | Duration to consider avoidance before passing by obstacles | 9.0           |
| drivable_area_generation.oncoming_object.end_duration_to_avoid        | [s]   | double | Duration to consider avoidance after passing by obstacles  | 0.0           |

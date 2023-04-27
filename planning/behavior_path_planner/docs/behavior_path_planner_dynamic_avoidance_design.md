# Dynamic avoidance design

This is a module designed for avoiding obstacles which are running.
Static obstacles such as parked vehicles are dealt with by the avoidance module.

## Purpose / Role

This module is designed for rule-based avoidance that is easy for developers to design its behavior. It generates avoidance path parameterized by intuitive parameters such as lateral jerk and avoidance distance margin. This makes it possible to pre-define avoidance behavior.

This module is under development.

### Parameters

| Name                                              | Unit  | Type   | Description                             | Default value |
| :------------------------------------------------ | :---- | :----- | :-------------------------------------- | :------------ |
| target_object.car                                 | [-]   | bool   | The flag whether to avoid cars or not   | true          |
| target_object.truck                               | [-]   | bool   | The flag whether to avoid trucks or not | true          |
| ...                                               | [-]   | bool   | ...                                     | ...           |
| target_object.min_obstacle_vel                    | [m/s] | double | Minimum obstacle velocity to avoid      | 1.0           |
| drivable_area_generation.lat_offset_from_obstacle | [m]   | double | Lateral offset to avoid from obstacles  | 0.8           |
| drivable_area_generation.time_to_avoid            | [s]   | double | Elapsed time for avoiding an obstacle   | 5.0           |
| drivable_area_generation.max_lat_offset_to_avoid  | [m]   | double | Maximum lateral offset to avoid         | 0.5           |

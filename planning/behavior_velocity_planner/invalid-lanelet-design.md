## Invalid Lanelet

### Role

This module plans the velocity of the related part of the path in case there is an invalid lanelet referring to it.

![invalid_lanelet_design.svg](docs%2Finvalid_lanelet%2Finvalid_lanelet_design.svg)

### Activation Timing

This function is activated when the lane id of the target path has an invalid lanelet label (i.e. the `invalid_lanelet` attribute is `yes`).

### Module Parameters

| Parameter           | Type   | Description                                               |
| ------------------- | ------ | --------------------------------------------------------- |
| `stop_margin`       | double | [m] margin for ego vehicle to stop before speed_bump      |
| `print_debug_info`  | bool   | whether debug info will be printed or not                 |


### Inner-workings / Algorithms

- Get invalid lanelet attribute on the path from lanelet2 map
- The invalid lanelet state machine starts in `INIT` state
- Get the intersection points between path and invalid lanelet polygon
- Assign the state to `APPROACHING` toward an invalid lanelet if:
  - the distance from front of the ego vehicle till the first interection point between the ego path and the invalid lanelet polygon is more than the `stop_margin`
- Assign the state to `INSIDE_INVALID_LANELET` if:
  - the first point of the ego path is inside the invalid lenelet polygon, or
  - the distance from front of the ego vehicle till the first interection point between the ego path and the invalid lanelet polygon is less than the `stop_margin`
- Assing the state to `STOPPED` when the vehicle is completeley stopped

![invalid_lanelet_scenarios.svg](docs%2Finvalid_lanelet%2Finvalid_lanelet_scenarios.svg)

- At each state, RTC settings are assigned according to the following table

#### RTC Settings During Different States
| State                    | RTC Activation   | Safe State | Distance                                                   |
| -----------------------  | ---------------- |------------|----------------------------------------------------------- |
| `INIT`                   | `false`          | `true`     | distance from ego front to first intersection point OR zero|
| `APPROACHING`            | `false`          | `true`     | distance from ego front to first intersection              |
| `INSIDE_INVALID_LANELET` | `false`          | `false`    | zero                                                       |
| `STOPPED`                | `true`           | `false`    | zero                                                       |


### Future Work

- Handle the case when the vehicle stops before an invalid lanelet but part of it footprint intersects with the invalid lanelet polygon.


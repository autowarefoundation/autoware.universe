## Invalid Lanelet

### Role

This module plans the velocity of the related part of the path in case there is an invalid lanelet referring to it.

An invalid lanelet is a lanelet that is out of operation design domain (ODD), i.e., the vehicle **must not** drive autonomously in this lanelet.  
A lanelet can be invalid (out of ODD) due to many motivations, either technical limitations of the SW and/or HW, business requirements, safety considerations, .... etc, or even a combination of those.

Some examples of Invalid Lanelets

- Closed road intentionally, due to construction work for example
- Underpass road that goes under a railway, for safety reasons
- Road with slope/inclination that the vehicle is not be able to drive autonomously due to technical limitations. And lots of other examples.

![invalid_lanelet_design.svg](docs%2Finvalid_lanelet%2Finvalid_lanelet_design.svg)

A lanelet becomes invalid by adding a new tag under the relevant lanelet in the map file `<tag k="invalid_lanelet" v="yes"/>`.

The target of this module is to stop the vehicle before entering the invalid lanelet (with configurable stop margin) or keep the vehicle stationary if autonomous mode started inside an invalid lanelet. Then ask the human driver to take the responsibility of the driving task (Request to Cooperate "RTC")

### Activation Timing

This function is activated when the lane id of the target path has an invalid lanelet label (i.e. the `invalid_lanelet` attribute is `yes`).

### Module Parameters

| Parameter          | Type   | Description                                          |
| ------------------ | ------ | ---------------------------------------------------- |
| `stop_margin`      | double | [m] margin for ego vehicle to stop before speed_bump |
| `print_debug_info` | bool   | whether debug info will be printed or not            |

### Inner-workings / Algorithms

- Get invalid lanelet attribute on the path from lanelet2 map
- The invalid lanelet state machine starts in `INIT` state
- Get the intersection points between path and invalid lanelet polygon
- Assign the state to `APPROACHING` toward an invalid lanelet if:
  - the distance from front of the ego vehicle till the first intersection point between the ego path and the invalid lanelet polygon is more than the `stop_margin`
- Assign the state to `INSIDE_INVALID_LANELET` if:
  - the first point of the ego path is inside the invalid lanelet polygon, or
  - the distance from front of the ego vehicle till the first intersection point between the ego path and the invalid lanelet polygon is less than the `stop_margin`
- Assign the state to `STOPPED` when the vehicle is completely stopped

![invalid_lanelet_scenarios.svg](docs%2Finvalid_lanelet%2Finvalid_lanelet_scenarios.svg)

- At each state, RTC settings are assigned according to the following table

#### RTC Settings During Different States

| State                    | RTC Activation | Safe State | Distance                                                    |
| ------------------------ | -------------- | ---------- | ----------------------------------------------------------- |
| `INIT`                   | `false`        | `true`     | distance from ego front to first intersection point OR zero |
| `APPROACHING`            | `false`        | `true`     | distance from ego front to first intersection               |
| `INSIDE_INVALID_LANELET` | `false`        | `false`    | zero                                                        |
| `STOPPED`                | `true`         | `false`    | zero                                                        |

### Future Work

- Handle the case when the vehicle stops before an invalid lanelet but part of it footprint intersects with the invalid lanelet polygon.

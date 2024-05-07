## Mission Remaining Distance and Time Calculator

### Role

This package aims to provide mission remaining distance and remaining time calculations

### Activation and Timing

- The calculations are activated once we have a route planned for a mission in Autoware
- The calculations are trigged timely based on the `update_rate` parameter

### Module Parameters

| Name          | Type   | Default Value | Explanation                 |
| ------------- | ------ | ------------- | --------------------------- |
| `update_rate` | double | 10.0          | Timer callback period. [Hz] |

### Inner-workings / Algorithms

- Remaining Distance Calculation

  - The remaining distance calculation is based on getting the remaining shortest path between thecurrent vehicle pose and goal pose using `lanelet2` routing APIs.
  - The remaining distance is calculated by summing the 2D length of remaining shortest path, with exception to current lanelet and goal lanelet
    - For the current lanelet, the distance is calculated from the current vehicle position to the end of that lanelet
    - For the goal lanelet, the distance is calculated from the start of the lanelet to the goal pose in this lanelet.

- Remaining Time Calculation
  - Remaining time is calculated using simple equation of motion by getting the current vehicle velocity magnitude.
  - Then the calclated remaining distance is divided by the velocity magnitude.

### Future Work

- Find a more efficient way for remaining distance calculation instead of regularly searching the graph for finding the remaining shortest path
- Engage more sophisticated motion models for more accurate remainig time calculations
- Handle cases when the vehicle stops during the mission or Autoware get disengaged then engaged for the same mission

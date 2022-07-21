# Lane Change

The Lane Change module is activated when lane change is needed and can be safely executed.

## Lane Change Requirement

- During lane change request condition
  - The ego-vehicle isn’t on a `preferred_lane`.
  - There is neither intersection nor crosswalk on the path of the lane change
- lane change ready condition
  - Path of the lane change does not collide with other dynamic objects (see the figure below)
  - Lane change candidate path is approved by an operator.

## Generating Lane Change Candidate Path

The lane change candidate path is divided into two phases: preparation and lane-changing. The following figure illustrates each phase of the lane change candidate path.

![lane-change-phases](./image/lane_change/lane_change-lane_change_phases.png)

### Preparation phase

The preparation trajectory is the candidate path's first and the straight portion generated along the ego vehicle's current lane. The length of the preparation trajectory is computed as follows.

```C++
lane_change_prepare_distance = max(current_speed * lane_change_prepare_duration + 0.5 * deceleration * lane_change_prepare_duration^2, minimum_lane_change_prepare_distance)
```

During the preparation phase, the turn signal will be activated when the remaining distance is equal to or less than `lane_change_search_distance`.

### Lane-changing phase

The lane-changing phase consist of the shifted path that moves ego from current lane to the target lane. Total distance of lane-changing phase is as follows.

```C++
lane_change_prepare_velocity = current_speed + deceleration * lane_change_prepare_duration
lane_changing_distance = max(lane_change_prepare_velocity * lane_changing_duration + 0.5 * deceleration * lane_changing_duration^2, minimum_lane_change_length + backward_length_buffer_for_end_of_lane)
```

## Parameters

### Essential lane change parameters

The following parameters are configurable in `behavior_path_planner.param.yaml`.

| Name                                     | Unit | Type   | Description                                                                           | Default value |
| :--------------------------------------- | ---- | ------ | ------------------------------------------------------------------------------------- | ------------- |
| `backward_length_buffer_for_end_of_lane` | [m]  | double | The end of lane buffer to ensure ego vehicle has enough distance to start lane change | 5.0           |
| `minimum_lane_change_length`             | [m]  | double | The minimum distance needed for changing lanes.                                       | 12.0          |

The following parameters are configurable in `lane_change.param.yaml`.

| Name                              | Unit    | Type    | Description                                                                             | Default value |
| :-------------------------------- | ------- | ------- | --------------------------------------------------------------------------------------- | ------------- |
| `lane_change_prepare_duration`    | [m]     | double  | The preparation time for the ego vehicle to be ready to perform lane change.            | 4.0           |
| `lane_changing_duration`          | [m]     | double  | The total time that is taken to complete the lane-changing task.                        | 8.0           |
| `lane_change_finish_judge_buffer` | [m]     | double  | The additional buffer used to confirm lane change process completion                    | 3.0           |
| `minimum_lane_change_velocity`    | [m/s]   | double  | Minimum speed during lane changing process.                                             | 5.6           |
| `prediction_time_resolution`      | [s]     | double  | Time resolution for object's path interpolation and collision check.                    | 0.5           |
| `maximum_deceleration`            | [m/s^2] | double  | Ego vehicle maximum deceleration when performing lane change.                           | 1.0           |
| `lane_change_sampling_num`        | [-]     | int     | Number of possible lane-changing trajectories that are being influenced by deceleration | 10            |
| `enable_abort_lane_change`        | [-]     | boolean | Enable abort lane change. (\*1)                                                         | false         |
| `use_all_predicted_path`          | [-]     | boolean | If false, use only the predicted path that has the maximum confidence.                  | false         |
| `lane_change_search_distance`     | [m]     | double  | The turn signal activation distance during the lane change preparation phase.           | 30.0          |

(\*1) If path is already approved, this doesn't cancel lane change.

### Collision checks during lane change

The following parameters are configurable in `behavior_path_planner.param.yaml`.

| Name                              | Unit    | Type   | Description                                                                                                                                                    | Default value |
| :-------------------------------- | ------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `lateral_distance_max_threshold`  | [m]     | double | The lateral distance threshold that is used to determine whether lateral distance between two object is enough and whether lane change is safe.                | 5.0           |
| `expected_front_deceleration`     | [m/s^2] | double | The front object's maximum deceleration when the front vehicle perform sudden braking. (\*2)                                                                   | -1.0          |
| `expected_rear_deceleration`      | [m/s^2] | double | The rear object's maximum deceleration when the rear vehicle perform sudden braking. (\*2)                                                                     | -1.0          |
| `rear_vehicle_reaction_time`      | [s]     | double | The reaction time of the rear vehicle driver which starts from the driver noticing the sudden braking of the front vehicle until the driver step on the brake. | 1.0           |
| `rear_vehicle_safety_time_margin` | [s]     | double | The time buffer for the rear vehicle to come into complete stop when its driver perform sudden braking.                                                        | 2.0           |

(\*2) the value must be negative.

### Debug

The following parameters are configurable in `lane_change.param.yaml`.

| Name                   | Unit | Type    | Description                  | Default value |
| :--------------------- | ---- | ------- | ---------------------------- | ------------- |
| `publish_debug_marker` | [-]  | boolean | Flag to publish debug marker | false         |

## Limitations

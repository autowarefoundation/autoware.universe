## Out of Lane

### Role

There are cases where the ego vehicle footprint goes out of the driving lane,
for example when taking a narrow turn with a large vehicle.
The `out_of_lane` module adds deceleration and stop points to the ego trajectory in order to prevent collisions from occuring in these "out of lane" cases.

### Activation Timing

This module is activated if `launch_out_of_lane_module` is set to true at launch.

### Inner-workings / Algorithms

The overall idea of this module is to calculate at which time does "out of lane" collisions with predicted objects occur.
The algorithm assumes the input ego trajectory contains accurate `time_from_start` values in order to calculate accurate time to collisions with the predicted object.

We will explain the inner-workings of the module by detailling the following steps:

1. Calculate the ego trajectory footprints.
2. Calculate the ego lanelets.
3. Calculate the "out of lane" areas.
4. Calculate the time to collisions (TTCs) at each "out of lane" area.
5. Calculate the stop or slowdown point ahead of the collision to avoid.

![overview](./docs/out_of_lane-footprints_other_lanes_overlaps_ranges.png)

#### 1. Ego trajectory footprints

In this first step, the ego footprint is projected at each trajectory point and its size is modified based on the `ego.extra_..._offset` parameters.

#### 2. Ego lanelets

In the second step, we calculate the lanelets followed by the ego trajectory.
We select all lanelets crossed by the trajectory linestring (sequence of trajectory points), as well as their preceding lanelets.

#### 3. Out of lane areas

Next, for each trajectory point, we create the corresponding "out of lane" areas by substracting the ego lanelets from the trajectory point footprint (from step 1).
Each area is associated with the lanelets overlapped by the area and with the corresponding ego trajectory point.

#### 4. Time to collisions

For each "out of lane" area, we calculate the times when a dynamic object will overlap the area based on its predicted paths.

In the case where parameter `mode` is set to `threshold` and the calculated time is less than `thrshold.time_threshold` parameter, then we decide to avoid the "out of lane" area.

In the case where parameter `mode` is set to `ttc`,
we calculate the time to collision by comparing the predicted time of the object with the `time_from_start` field contained in the trajectory point.
If the time to collision is bellow the `ttc.threshold` parameter value, we decide to avoid the "out of lane" area.

#### 5. Calculate the stop or slowdown point

There can be multiple points to avoid

Whether it is decided to slow down or stop is determined by the distance between the ego vehicle and the trajectory point to avoid.
If this distance is bellow the `actions.slowdown.threshold`, a velocity of `actions.slowdown.velocity` will be used.
If the distance is bellow the `actions.stop.threshold`, a velocity of `0`m/s will be used.

<table width="100%">
  <tr>
  <td>
    <img src="./docs/out_of_lane_slow.png" width="550px"/>
  </td>
  <td>
    <img src="./docs/out_of_lane_stop.png" width="550px"/>
  </td>
  </tr>
</table>

##### Threshold

With the `mode` set to `"threshold"`,
a decision to stop or slow down before a range is made if
an incoming dynamic object is estimated to reach the overlap within `threshold.time_threshold`.

##### TTC (time to collision)

With the `mode` set to `"ttc"`,
estimates for the times when ego and the dynamic objects reach the start and end of the overlapping range are calculated.
This is then used to calculate the time to collision over the period where ego crosses the overlap.
If the time to collision is predicted to go bellow the `ttc.threshold`, the decision to stop or slow down is made.

##### Intervals

With the `mode` set to `"intervals"`,
the estimated times when ego and the dynamic objects reach the start and end points of
the overlapping range are used to create time intervals.
These intervals can be made shorter or longer using the
`intervals.ego_time_buffer` and `intervals.objects_time_buffer` parameters.
If the time interval of ego overlaps with the time interval of an object, the decision to stop or slow down is made.

##### Time estimates

###### Ego

To estimate the times when ego will reach an overlap, it is assumed that ego travels along its path
at its current velocity or at half the velocity of the path points, whichever is higher.

###### Dynamic objects

Two methods are used to estimate the time when a dynamic objects with reach some point.
If `objects.use_predicted_paths` is set to `true`, the predicted paths of the dynamic object are used if their confidence value is higher than the value set by the `objects.predicted_path_min_confidence` parameter.
Otherwise, the lanelet map is used to estimate the distance between the object and the point and the time is calculated assuming the object keeps its current velocity.

#### 5. Path update

Finally, for each decision to stop or slow down before an overlapping range,
a point is inserted in the path.
For a decision taken for an overlapping range with a lane $l$ starting at ego path point index $i$,
a point is inserted in the path between index $i$ and $i-1$ such that the ego footprint projected at the inserted point does not overlap $l$.
Such point with no overlap must exist since, by definition of the overlapping range,
we know that there is no overlap at $i-1$.

If the point would cause a higher deceleration than allowed by the `max_accel` parameter (node parameter),
it is skipped.

Moreover, parameter `action.distance_buffer` adds an extra distance between the ego footprint and the overlap when possible.

### Module Parameters

| Parameter                     | Type   | Description                                                                       |
| ----------------------------- | ------ | --------------------------------------------------------------------------------- |
| `mode`                        | string | [-] mode used to consider a dynamic object. Candidates: threshold, intervals, ttc |
| `skip_if_already_overlapping` | bool   | [-] if true, do not run this module when ego already overlaps another lane        |

| Parameter /threshold | Type   | Description                                                      |
| -------------------- | ------ | ---------------------------------------------------------------- |
| `time_threshold`     | double | [s] consider objects that will reach an overlap within this time |

| Parameter /ttc | Type   | Description                                                                                            |
| -------------- | ------ | ------------------------------------------------------------------------------------------------------ |
| `threshold`    | double | [s] consider objects with an estimated time to collision bellow this value while ego is on the overlap |

| Parameter /objects              | Type   | Description                                                           |
| ------------------------------- | ------ | --------------------------------------------------------------------- |
| `minimum_velocity`              | double | [m/s] ignore objects with a velocity lower than this value            |
| `predicted_path_min_confidence` | double | [-] minimum confidence required for a predicted path to be considered |

| Parameter /overlap | Type   | Description                                                           |
| ------------------ | ------ | --------------------------------------------------------------------- |
| `minimum_distance` | double | [m] minimum distance inside a lanelet for an overlap to be considered |

| Parameter /action             | Type   | Description                                                           |
| ----------------------------- | ------ | --------------------------------------------------------------------- |
| `slowdown.distance_threshold` | double | [m] insert a slow down when closer than this distance from an overlap |
| `slowdown.velocity`           | double | [m] slow down velocity                                                |
| `stop.distance_threshold`     | double | [m] insert a stop when closer than this distance from an overlap      |

| Parameter /ego       | Type   | Description                                          |
| -------------------- | ------ | ---------------------------------------------------- |
| `extra_front_offset` | double | [m] extra front distance to add to the ego footprint |
| `extra_rear_offset`  | double | [m] extra rear distance to add to the ego footprint  |
| `extra_left_offset`  | double | [m] extra left distance to add to the ego footprint  |
| `extra_right_offset` | double | [m] extra right distance to add to the ego footprint |

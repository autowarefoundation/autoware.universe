## Run Out

### Role

`out_of_lane` is the module that decelerates and stops to prevent the ego vehicle from entering another lane with incoming dynamic objects.

![brief](./docs/out_of_lane/out_of_lane-overview.svg)

### Activation Timing

This module is activated if `launch_out_of_lane` is set to true

### Inner-workings / Algorithms

The algorithm is made of the following steps.
1. Calculate the ego path footprints.
1. Calculate the other lanes.
1. Calculate the overlapping ranges between the ego path footprints and the other lanes.
1. For each overlapping range, decide if a stop or slow down action must be taken.
1. For each action, insert the corresponding stop or slow down point in the path.

#### 1. Ego Path Footprints

In this first step, the ego footprint is projected at each path point and are eventually inflated based on the `extra_..._offset` parameters.

#### 2. Other lanes

In the second step, the set of lanes to consider for overlaps is generated.
This set is built by selecting all lanelets within some distance from the ego vehicle, and then removing non-relevent lanelets.
The selection distance is choosen as the maximum between the `slowdown.distance_threshold` and the `stop.distance_threshold`.

A lanelet is deemed non-relevent if it meets one of the following conditions.
- It is part of the lanelets followed by the ego path.
- It contains the rear point of the ego footprint.
- It follows one of the ego path lanelets.

#### 3. Overlapping ranges

In the third step, overlaps between the ego path footprints and the other lanes are calculated.
For each pair of other lane $l$ and ego path footprint $f$, we calculate the overlapping polygons using `boost::geometry::intersection`.
For each overlapping polygon found, if the distance inside the other lane $l$ is above the `overlap.minimum_distance` threshold, then the overlap is ignored.
Otherwise, the arc length range (relative to the ego path) and corresponding points of the overlapping polygons are stored.
Ultimately, for each other lane $l$, overlapping ranges of successive overlaps are built with the following information:
- overlapped other lane $l$.
- start and end ego path indexes.
- start and end ego path arc lengths.
- start and end overlap points.

#### 4. Decisions

In the fourth step, a decision to either slow down or stop before each overlapping range is taken based on the dynamic objects.
The conditions for the decision depend on the value of the `mode` parameter.

##### Threshold

##### Intervals

##### TTC (time to collision)

##### Dynamic objects 

Two methods are used to estimate the time when a dynamic objects with reach some point.
If `objects.use_predicted_paths` is set to `true`, the predicted path of the dynamic object is used.
Otherwise, the lanelet map is used to estimate the distance between the object and the point and the time is calculated assuming the object keeps its current velocity.

#### 5. Path update

For each decision to stop or slow down before an overlapping interval with other lane $l$ starting at ego path point index $i$,
a point is inserted in the path between index $i$ and $i-1$ such that the ego footprint projected at the inserted point does not overlap $l$.

### Module Parameters

| Parameter               | Type   | Description                                                                                                              |
| ----------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------ |
| `mode`      | string | [-] mode used to consider a dynamic object. Candidates: threshold, intervals, ttc |
| `skip_if_already_overlapping` | bool   | [-] if true, do not run this module when ego already overlaps another lane |

| Parameter /threshold | Type   | Description                                  |
| ------------------------- | ------ | -------------------------------------------- |
| `time_threshold`            | double | [s] consider objects that will reach an overlap within this time |

| Parameter /intervals | Type   | Description                                                                                                                   |
| --------------------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------- |
| `ego_time_buffer`              | double | [s] extend the ego time interval by this buffer |
| `objects_time_buffer`              | double | [s] extend the time intervals of objects by this buffer |

| Parameter /ttc | Type   | Description                                           |
| ---------------------- | ------ | ----------------------------------------------------- |
| `threshold`               | double | [s] consider objects with an estimated time to collision bellow this value while ego is on the overlap |

| Parameter /objects | Type   | Description                                                                         |
| ------------------------ | ------ | ----------------------------------------------------------------------------------- |
| `minimum_velocity`            | double | [m/s] consider objects with an estimated time to collision bellow this value while on the overlap |
| `use_predicted_paths`       | bool | [-] if true, use the predicted paths to estimate future positions; if false, assume the object moves at constant velocity along *all* lanelets it currently is located in |

| Parameter /overlap | Type   | Description                                                   |
| -------------------------- | ------ | ------------------------------------------------------------- |
| `minimum_distance`                   | double | [m] minimum distance inside a lanelet for an overlap to be considered |
| `extra_length`                 | double | [m] extra arc length to add to the front and back of an overlap (used to calculate enter/exit times) |

| Parameter /action | Type   | Description                                                   |
| -------------------------- | ------ | ------------------------------------------------------------- |
| `skip_if_over_max_decel`                   | bool   | [-] if true, do not take an action that would cause more deceleration than the maximum allowed |
| `strict`                 | bool | [-] if true, when a decision is taken to avoid entering a lane, the stop point will make sure no lane at all is entered by ego; if false, ego stops just before entering a lane but may then be overlapping another lane |
| `distance_buffer`                 | double | [m] buffer distance to try to keep between the ego footprint and lane |
| `slowdown.distance_threshold`                 | double | [m] insert a slow down when closer than this distance from an overlap |
| `slowdown.velocity`                 | double | [m] slow down velocity |
| `stop.distance_threshold`                 | double | [m] insert a stop when closer than this distance from an overlap |

| Parameter /ego | Type   | Description                                                   |
| -------------------------- | ------ | ------------------------------------------------------------- |
| `extra_front_offset`       | double | [m] extra front distance to add to the ego footprint |
| `extra_rear_offset`       | double | [m] extra rear distance to add to the ego footprint |
| `extra_left_offset`       | double | [m] extra left distance to add to the ego footprint |
| `extra_right_offset`       | double | [m] extra right distance to add to the ego footprint |

### Future extensions / Unimplemented parts
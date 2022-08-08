# Motion Utils package

## Nearest index search

In this section, nearest index and nearest segment index search is explained.

We have the same functions for nearest index search and nearest segment index search.
Taking for the example the nearest index, we have two types of functions as follows.

```cpp
template <class T>
size_t findFirstNearestIndexWithSoftConstraints(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());
```

This function finds the neareset index with distance and yaw thresholds.
There are default parameters for these thresholds.

1. When both distance and yaw thresholds are given.
   - First, try to find the nearest index with both the distance and yaw thresholds.
   - If not found, try to find again with only the distance threshold.
   - If not found, find without any thresholds.
2. When only distance are given.
   - First, try to find the nearest index the distance threshold.
   - If not found, find without any thresholds.
3. When no thresholds are given.
   - Find the nearest index.

```cpp
template <class T>
size_t findNearestIndexWithinRange(
  const T & points, const geometry_msgs::msg::Point & pos, const size_t start_idx,
  const size_t end_idx);
```

This functions finds the nearest index within the range from `start_idx` to `end_idx`, usually used with another function calculating the range.

### Application to various object

Many node packages often calculate the nearest index of objects.
We will explain the recommended method to calculate it.

#### Nearest index for the ego

Assuming that the path length before the ego is short enough, we expect to find the correct nearest index in the following cases by `findFirstNearestIndexWithSoftConstraints` with both distance and yaw thresholds.
Blue circles describes the distance threshold from the base link position and blue lines describe the yaw threshold against the base link orientation.
Among points in these cases, the correct nearest point which is red can be found.

![ego_nearest_search](./media/ego_nearest_search.svg)

Therefore, the implementation is as follows.

```cpp
const size_t ego_nearest_idx = findFirstNearestIndex(points, ego_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
const size_t ego_nearest_seg_idx = findFirstNearestIndex(points, ego_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
```

#### Nearest index for dynamic objects

For the ego nearest index, the orientation is considered in addition to the position since the ego follows the points.
However, for the dynamic objects, sometimes the orientation may be different from the points order, e.g. the dynamic object driving backward algthough the ego is driving forward.

Therefore, the yaw threshold should not be considered for the dynamic object.
The impelementation is as follows.

```cpp
const size_t ego_nearest_idx = findFirstNearestIndex(points, traffic_obj_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
const size_t ego_nearest_seg_idx = findFirstNearestIndex(points, traffic_obj_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
```

## Path/Trajectory length calculation between designated points.

Based on the discussion so far, the nearest search algorithm is different depending on the object type.
Therefore, we recommended to use the wrapper functions like calculating the path length with the nearest segment index search functions.

For example, when we want to calculate the path length between the ego and the dynamic object, it will be like this.

```cpp
const size_t ego_nearest_idx = findFirstNearestSegmentIndex(points, ego_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
const size_t dyn_obj_nearest_seg_idx = findFirstNearestSegmentIndex(points, dyn_obj_pose, dyn_obj_nearest_dist_threshold);
const double length_from_ego_to_obj = calcSignedArcLength(points, ego_pose, ego_nearest_seg_idx, dyn_obj_pose, dyn_obj_nearest_seg_idx);
```


# Overview
Lane Change Planner should create path from Route message, and navigates to given goal by either following lane or changing lanes. 

## Inputs
Input : topic "route" with type autoware_planning_msgs::Route

## Outputs
topic name:  "path_with_lane_id"
type: autoware_planning_msgs::PathWithLaneId
frequency: 10Hz

## Assumptions
* Vehicle travels at constant speed

## Requirements
### About goal pose
* If the given goal pose in route message is within route, then goal pose is used directly
* If the given goal pose in route message is not within route (e.g. in parking space), then the goal shall be replaced by closest point along the centerline of goal lane

### About Drivable Area
* drivable area should be the shape of lanes that vehicle is driving when BPP is following lane
* drivable area should be the shape of current lane and lane change target lanes when BPP is operating lane change 

### About Path Points
* LaneChanePlanner should publish reference path that leads to goal pose
* Path should start from n [m] behind vehicle position
* Path should have length of at least 100[m] unless path surpasses goal
* All points in Path should be placed within drivable area
* Path within the n [m] away from vehicle should not change over time to avoid sudden change in steering.

### About Lane Change
* Vehicle follows lane if vehicle is on preferred lanes
* Vehicle should stay in lane at least for 3 second before operating lane change for other participants to recognize ego vehicle's turn signal.
* The planner attempts lane change towards preferred lane if vehicle is not within preferred lanes, and candidate lane change path is valid. The path is valid if it satisfies all the following conditions:
  * there is 2seconds margin between any other vehicles assuming that ego vehicle follows the candidate path at constant speed
  * lane change finishes x [m] before any intersections
  * lane change finishes x [m] before any crosswalks
* LaneChangePlanner shall abort lane change and go back to original lane when all of the following conditions are satisfied:
  * Vehicle(base_link) is still in the original lane
  * there is no n seconds margin between all other vehicles during lane change, assuming that ego vehicle follows the candidate path at constant speed. (due to newly detected vehicles)
RecordReplay planner {#recordreplay-planner}
====================

# Purpose / Use cases

This package provides a simple class for recording states, then playing them back as a trajectory. This is to
be used in both the simulator as well as in on-site tests of the controller: One can turn on recording, start
driving a trajectory, then reset vehicle position, then start replay and see what the controller does.

The trajectories can be a loop , which can be determined naively by checking the distance between the first and last
point of the trajectory, but the user has the ultimate control over toggling this behavior.


# Design

Recording will add states at the end of an internal list of states.

The replay will find the closest state in terms of location and heading along the recorded list of states, and
deliver trajectories starting from that state. The trajectory length is at most 100 as specified by the
`Trajectory` message, and at least 1 if there is any recorded data present.

No obstacle collision is checked when replaying. The produced `Trajectory` message, however, can be fed to an `obstacle_collision_estimator` node which will modify the trajectory considering the obstacles. See the `recordreplay_planner_nodes` design documentation for more details.

## Assumptions / Known limits

There is no interpolation between points along the trajectory, and localization is not done in a smart way:
The list of recorded states is simply iterated over.

The stopping concept only works if one can assume that the downstream controller and the vehicle are able
to track the desired velocity going to zero in a single trajectory step. 

Making sure this assumption is satisfied by construction would involve creating a dynamically feasible
velocity profile for stopping - this has not been done yet. 

It is the user's responsibility to check if the trajectory is suitable for looping, and call `set_loop` accordingly
to toggle the looping behavior with regard to the current trajectory that the controller is replaying. `is_loop`,
which is a naive function to check if the start and end point of a trajectory is sufficiently closed enough, is provided
and in fact used by `recordreplay_planner_nodes`.

When looping reaches the last point, it simply concatenates the beginning of the trajectory. No effort is made to
ensure that the position, orientation, and speed updates are kinematically feasible. For example, if during recording
the vehicle stopped at the last point, the trajectory will contain the same speed information, even though the vehicle
is supposed to keep driving to the first point. Therefore, to use looping, it is highly recommended to first optimize
the path file that the user obtained while recording.

## Inputs / Outputs / API

Inputs:

* `VehicleKinematicState.msg` is the state that gets recorded
* `BoundingBoxArray.msg` is a list of bounding boxes of obstacles

Outputs:

* `Trajectory.msg` is the trajectory that gets published


## Complexity

Recording is `O(1)` in time and `O(n)` in space, replay is `O(n)` in both time and space, where `n` is the
number of recorded states. Collision checking currently happens on every replay even if obstacles do not
change, and has a complexity that is linear in the number of obstacles but proportional to the product of 
the number of halfplanes in the ego vehicle and a single obstacle.

# Security considerations 

TBD by a security specialist.

# Future extensions / Unimplemented parts

* Trajectory buffer clearing
* Proper `tf` support
* rosbag2 support for recording and later replaying

# Related issues


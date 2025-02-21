# Planning Validator

The `autoware_planning_validator` is a module that checks the validity of a trajectory before it is published. The status of the validation can be viewed in the `/diagnostics` and `/validation_status` topics. When an invalid trajectory is detected, the `autoware_planning_validator` will process the trajectory following the selected option: "0. publish the trajectory as it is", "1. stop publishing the trajectory", "2. publish the last validated trajectory".

![autoware_planning_validator](./image/planning_validator.drawio.svg)

## Supported features

The following features are supported for trajectory validation and can have thresholds set by parameters:

- **Invalid field** : e.g. Inf, Nan
- **Trajectory points interval** : invalid if any of the distance of trajectory points is too large
- **Curvature** : invalid if the trajectory has too sharp turns that is not feasible for the given vehicle kinematics
- **Relative angle** : invalid if the yaw angle changes too fast in the sequence of trajectory points
- **Lateral acceleration** : invalid if the expected lateral acceleration/deceleration is too large
- **Longitudinal acceleration/deceleration** : invalid if the acceleration/deceleration in the trajectory point is too large
- **Steering angle** : invalid if the expected steering value is too large estimated from trajectory curvature
- **Steering angle rate** : invalid if the expected steering rate value is too large
- **Velocity deviation** : invalid if the planning velocity is too far from the ego velocity
- **Distance deviation** : invalid if the ego is too far from the trajectory
- **Longitudinal distance deviation** : invalid if the trajectory is too far from ego in longitudinal direction
- **Forward trajectory length** : invalid if the trajectory length is not enough to stop within a given deceleration

The following features are to be implemented.

- **(TODO) TTC calculation** : invalid if the expected time-to-collision is too short on the trajectory

## Inputs/Outputs

### Inputs

The `autoware_planning_validator` takes in the following inputs:

| Name                 | Type                              | Description                                    |
| -------------------- | --------------------------------- | ---------------------------------------------- |
| `~/input/kinematics` | nav_msgs/Odometry                 | ego pose and twist                             |
| `~/input/trajectory` | autoware_planning_msgs/Trajectory | target trajectory to be validated in this node |

### Outputs

It outputs the following:

| Name                         | Type                                       | Description                                                               |
| ---------------------------- | ------------------------------------------ | ------------------------------------------------------------------------- |
| `~/output/trajectory`        | autoware_planning_msgs/Trajectory          | validated trajectory                                                      |
| `~/output/validation_status` | planning_validator/PlanningValidatorStatus | validator status to inform the reason why the trajectory is valid/invalid |
| `/diagnostics`               | diagnostic_msgs/DiagnosticStatus           | diagnostics to report errors                                              |

## Parameters

{{ json_to_markdown("planning/autoware_planning_validator/schema/planning_validator.schema.json") }}

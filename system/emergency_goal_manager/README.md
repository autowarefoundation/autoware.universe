# emergency_goal_manager

## Purpose

The Emergency goal manager is responsible for coordinating the goal poses for emergency rerouting and communicating it to the mission planner.

## Inner-workings / Algorithms

TBD.

## Inputs / Outputs

### Input

| Name                                    | Type                                                 | Description                   | 
| --------------------------------------- | ---------------------------------------------------- | ----------------------------- | 
| `~/input/emergency_goals`               | `tier4_system_msgs::msg::EmergencyGoalsStamped`      | Candidates for emergency goal | 
| `~/input/emergency_goals_clear_command` | `tier4_system_msgs::msg::EmergencyGoalsClearCommand` | Clear command                 | 

### Output

| Name                                                             | Type                                          | Description              | 
| ---------------------------------------------------------------- | --------------------------------------------- | ------------------------ | 
| `/planning/mission_planning/mission_planner/srv/set_mrm_route`   | `autoware_adapi_v1_msgs::srv::SetRoutePoints` | Set route points for MRM | 
| `/planning/mission_planning/mission_planner/srv/clear_mrm_route` | `autoware_adapi_v1_msgs::srv::ClearRoute`     | Clear route for MRM      | 

## Parameters

No parameters.

## Assumptions / Known limits

TBD.

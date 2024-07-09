# HMI Node

Creates a mission based on the terminal input (ROS parameter change).

Available missions:

- LANE_KEEP
- LANE_CHANGE_LEFT
- LANE_CHANGE_RIGHT
- TAKE_NEXT_EXIT_LEFT
- TAKE_NEXT_EXIT_RIGHT

Interact with this node by changing the ROS parameters. For a lane change to the right use this command in the terminal:

```bash
ros2 param set /mission_planner/hmi mission LANE_CHANGE_RIGHT
```

## Output topics

| Name                      | Type                                 | Description |
| ------------------------- | ------------------------------------ | ----------- |
| `hmi_node/output/mission` | autoware_planning_msgs::msg::Mission | mission     |

## Node parameters

| Parameter | Type   | Description                                    |
| --------- | ------ | ---------------------------------------------- |
| `mission` | string | the mission (LANE_KEEP, LANE_CHANGE_LEFT, ...) |

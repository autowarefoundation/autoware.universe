# Local Map Provider Node

This node converts the mission_planner_messages::msg::RoadSegments message into a mission_planner_messages::msg::LocalMap message right now. More functionality can be added later.

## Input topics

| Name                                          | Type                                      | Description   |
| --------------------------------------------- | ----------------------------------------- | ------------- |
| `local_map_provider_node/input/road_segments` | autoware_planning_msgs::msg::RoadSegments | road segments |

## Output topics

| Name                                       | Type                                  | Description |
| ------------------------------------------ | ------------------------------------- | ----------- |
| `local_map_provider_node/output/local_map` | autoware_planning_msgs::msg::LocalMap | local map   |

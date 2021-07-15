# Scenario Selector

## Role

The role of scenario selector is to select appropriate scenario planner depending on situation. For example, if current pose is within road, then scenario selector should choose on-road planner, and if vehicle is within parking lot, then scenario selector should choose parking scenario.

### Input

- map: `autoware_lanelet_msgs::MapBin`
- vehicle pose: `/tf` (map->base_link)
- route: `autoware_planning_msgs::Route` <br> Scenario planner uses above three topics to decide which scenario to use. In general it should decide scenarios based on where in the map vehicle is located(map+vehicle pose) and where it is trying to go(route).
- trajectory: `autoware_planning_msgs::Trajectory` <br> Scenario planner gets the output from all the scenarios and passes the trajectory from selected scenario down to following stacks. This must be done within scenario_selector module in order to sync with the timing of scenario changing.

### Output

- scenario: `autoware_planning_msgs::Scenario` <br> This contains current available scenario and selected scenario. Each Scenario modules read this topic and chooses to plan trajectory
- Trajectory: `autoware_planning_msgs::Trajectory` <br> This is the final trajectory of Planning stack, which is the trajectory from selected Scenario module.
